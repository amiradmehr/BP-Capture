import sys
import serial
import time
import multiprocessing
from datetime import datetime
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import csv

# Serial configuration (adjust COM port as per your Arduino)
SERIAL_PORT = 'COM5'  # Change this if needed
BAUD_RATE = 115200

# Sampling interval (in milliseconds)
SAMPLE_INTERVAL = 10

def read_serial_data(data_queue, running_event, connection_status, command_queue):
    """Process that reads data from the serial port and sends commands from the command queue."""
    arduino_serial = None
    start_time = time.time()

    while True:
        # Attempt to connect if not connected
        if arduino_serial is None or not arduino_serial.is_open:
            try:
                arduino_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                time.sleep(2)  # Allow time for connection initialization
                start_time = time.time()  # Reset baseline time
                connection_status.value = 1
                print("Connected to Arduino.")
            except serial.SerialException as e:
                connection_status.value = 0
                print("Failed to connect to Arduino. Retrying...", e)
                time.sleep(1)
                continue

        # Send any commands waiting in the queue
        try:
            while not command_queue.empty():
                command = command_queue.get()
                arduino_serial.write((command + "\n").encode('utf-8'))
                print("Sent command:", command)
        except Exception as e:
            print("Error sending command:", e)

        # Read data if running_event is set
        try:
            if running_event.is_set() and arduino_serial.in_waiting > 0:
                line = arduino_serial.readline().decode('utf-8', errors='replace').strip()
                if line.startswith("ADC Reading:"):
                    line = line[len("ADC Reading:"):].strip()
                try:
                    adc_value = float(line)
                    elapsed_time_ms = int((time.time() - start_time) * 1000)
                    print(f"Time (ms): {elapsed_time_ms}, ADC Value: {adc_value}")
                    data_queue.put((elapsed_time_ms, adc_value))
                except ValueError:
                    # Print unexpected line for debugging
                    print("Received non-numeric data:", line)
        except (serial.SerialException, OSError) as e:
            print("Lost connection to Arduino. Attempting to reconnect...", e)
            connection_status.value = 0
            if arduino_serial:
                arduino_serial.close()
            arduino_serial = None
            time.sleep(1)

class RealTimePlotter(QtWidgets.QMainWindow):
    def __init__(self, data_queue, running_event, connection_status, command_queue):
        super().__init__()
        self.data_queue = data_queue
        self.running_event = running_event
        self.connection_status = connection_status
        self.command_queue = command_queue
        self.check_connection_timer = QtCore.QTimer()
        
        # CSV file attributes
        self.csv_file = None
        self.csv_writer = None
        self.stream_start_time = None

        # Set up the GUI layout
        self.setWindowTitle("Real-Time ADC Data Plotter and Motor Control")
        self.setGeometry(100, 100, 800, 600)

        # Main layout
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        layout = QtWidgets.QVBoxLayout(main_widget)

        # Plot setup
        self.plot_widget = pg.PlotWidget(background='w')
        layout.addWidget(self.plot_widget)
        self.plot_widget.setLabel('left', 'ADC Value')
        self.plot_widget.setLabel('bottom', 'Time (ms)')

        self.time_data = []
        self.adc_data = []
        self.curve = self.plot_widget.plot(self.time_data, self.adc_data, pen='k')

        self.time_window = 5000  # last 5 seconds
        self.y_axis_range = None

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(SAMPLE_INTERVAL)

        # Controls Layout
        control_layout = QtWidgets.QHBoxLayout()
        y_axis_label = QtWidgets.QLabel("Y-Axis Scale")
        self.y_axis_combo = QtWidgets.QComboBox()
        self.y_axis_combo.addItems(["Auto", "0-100", "-100 to 100", "0-200"])
        self.y_axis_combo.currentIndexChanged.connect(self.change_y_axis_scale)
        control_layout.addWidget(y_axis_label)
        control_layout.addWidget(self.y_axis_combo)

        x_axis_label = QtWidgets.QLabel("X-Axis Time Window (ms)")
        self.x_axis_combo = QtWidgets.QComboBox()
        self.x_axis_combo.addItems(["1 sec", "5 sec", "10 sec", "30 sec"])
        self.x_axis_combo.currentIndexChanged.connect(self.change_time_window)
        control_layout.addWidget(x_axis_label)
        control_layout.addWidget(self.x_axis_combo)

        self.start_stop_button = QtWidgets.QPushButton("Start Streaming")
        self.start_stop_button.setCheckable(True)
        self.start_stop_button.clicked.connect(self.toggle_streaming)
        control_layout.addWidget(self.start_stop_button)

        self.status_label = QtWidgets.QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("color: red;")
        control_layout.addWidget(self.status_label)

        layout.addLayout(control_layout)

        # Motor Control Buttons Layout
        motor_layout = QtWidgets.QHBoxLayout()
        motor_label = QtWidgets.QLabel("Motor Control:")
        motor_layout.addWidget(motor_label)

        self.forward_button = QtWidgets.QPushButton("Forward")
        self.forward_button.clicked.connect(self.send_forward)
        motor_layout.addWidget(self.forward_button)

        self.backward_button = QtWidgets.QPushButton("Backward")
        self.backward_button.clicked.connect(self.send_backward)
        motor_layout.addWidget(self.backward_button)

        self.slow_forward_button = QtWidgets.QPushButton("Slow Forward (60 sec)")
        self.slow_forward_button.clicked.connect(self.send_slow_forward)
        motor_layout.addWidget(self.slow_forward_button)

        self.stop_button = QtWidgets.QPushButton("Stop Motor")
        self.stop_button.clicked.connect(self.send_stop)
        motor_layout.addWidget(self.stop_button)

        layout.addLayout(motor_layout)

        self.check_connection_timer.timeout.connect(self.update_connection_status)
        self.check_connection_timer.start(1000)

    def change_y_axis_scale(self):
        y_scale = self.y_axis_combo.currentText()
        if y_scale == "Auto":
            self.y_axis_range = None
            self.plot_widget.enableAutoRange(axis=pg.ViewBox.YAxis)
        elif y_scale == "0-100":
            self.y_axis_range = (0, 100)
            self.plot_widget.setYRange(*self.y_axis_range)
        elif y_scale == "-100 to 100":
            self.y_axis_range = (-100, 100)
            self.plot_widget.setYRange(*self.y_axis_range)
        elif y_scale == "0-200":
            self.y_axis_range = (0, 200)
            self.plot_widget.setYRange(*self.y_axis_range)

    def change_time_window(self):
        x_window = self.x_axis_combo.currentText()
        if x_window == "1 sec":
            self.time_window = 1000
        elif x_window == "5 sec":
            self.time_window = 5000
        elif x_window == "10 sec":
            self.time_window = 10000
        elif x_window == "30 sec":
            self.time_window = 30000

    def toggle_streaming(self):
        if self.start_stop_button.isChecked():
            if self.connection_status.value == 1:
                self.running_event.set()
                self.start_stop_button.setText("Stop Streaming")
                now = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"adcreading{now}.csv"
                try:
                    self.csv_file = open(filename, "w", newline="")
                    self.csv_writer = csv.writer(self.csv_file)
                    self.csv_writer.writerow(["Time (ms)", "ADC Value"])
                    print(f"Streaming started. Saving data to {filename}")
                except Exception as e:
                    print("Error opening CSV file:", e)
                self.stream_start_time = None
            else:
                self.running_event.clear()
                self.start_stop_button.setText("Start Streaming")
                if self.csv_file:
                    self.csv_file.close()
                    self.csv_file = None
                    self.csv_writer = None
                    print("Streaming stopped. CSV file closed.")
        else:
            self.running_event.clear()
            self.start_stop_button.setText("Start Streaming")
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
                print("Streaming stopped. CSV file closed.")

    def update_connection_status(self):
        if self.connection_status.value == 1:
            self.status_label.setText("Status: Connected")
            self.status_label.setStyleSheet("color: green;")
        else:
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet("color: red;")
            self.running_event.clear()
            self.start_stop_button.setChecked(False)
            self.start_stop_button.setText("Start Streaming")

    def update_plot(self):
        while not self.data_queue.empty():
            time_ms, adc_value = self.data_queue.get()
            self.time_data.append(time_ms)
            self.adc_data.append(adc_value)

            while self.time_data and (time_ms - self.time_data[0]) > self.time_window:
                self.time_data.pop(0)
                self.adc_data.pop(0)

            self.plot_widget.setXRange(max(0, time_ms - self.time_window), time_ms)
            self.curve.setData(self.time_data, self.adc_data)

            if self.csv_writer is not None:
                if self.stream_start_time is None:
                    self.stream_start_time = time_ms
                adjusted_time = time_ms - self.stream_start_time
                try:
                    self.csv_writer.writerow([adjusted_time, adc_value])
                except Exception as e:
                    print("Error writing to CSV:", e)

    # --- Motor Command Methods ---
    def send_forward(self):
        self.command_queue.put("]")
        print("Forward command sent.")

    def send_backward(self):
        self.command_queue.put("[")
        print("Backward command sent.")

    def send_slow_forward(self):
        self.command_queue.put("k")
        print("Slow Forward (60 sec) command sent.")

    def send_stop(self):
        self.command_queue.put("x")
        print("Stop command sent.")

def main():
    data_queue = multiprocessing.Queue()
    running_event = multiprocessing.Event()
    connection_status = multiprocessing.Value('i', 0)
    command_queue = multiprocessing.Queue()

    serial_process = multiprocessing.Process(target=read_serial_data,
                                              args=(data_queue, running_event, connection_status, command_queue))
    serial_process.start()

    app = QtWidgets.QApplication(sys.argv)
    main_window = RealTimePlotter(data_queue, running_event, connection_status, command_queue)
    main_window.show()

    try:
        sys.exit(app.exec_())
    finally:
        running_event.clear()
        serial_process.terminate()
        serial_process.join()

if __name__ == "__main__":
    main()
