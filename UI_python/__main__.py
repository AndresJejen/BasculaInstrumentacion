import sys
import serial
import csv
from datetime import datetime
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QWidget, QLabel, QMessageBox)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal

class SerialReader(QThread):
    # Signal to emit when new data is received
    data_received = pyqtSignal(float)
    error_occurred = pyqtSignal(str)

    def __init__(self, port, baud_rate):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.serial_port = None
        self.is_running = False

    def run(self):
        try:
            # Open serial port
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.is_running = True

            while self.is_running:
                try:
                    # Check if data is available
                    if self.serial_port.in_waiting > 0:
                        # Read a line and try to convert to float
                        line = self.serial_port.readline().decode('utf-8').strip()
                        try:
                            value = float(line)
                            # Emit the received data
                            self.data_received.emit(value)
                        except ValueError:
                            # If conversion fails, emit an error
                            self.error_occurred.emit(f"Invalid data: {line}")
                except Exception as read_error:
                    self.error_occurred.emit(f"Serial read error: {read_error}")
                    break

        except serial.SerialException as e:
            self.error_occurred.emit(f"Could not open serial port: {e}")
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

    def stop(self):
        self.is_running = False
        self.wait()

class ArduinoDataPlotter(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Try to detect the correct serial port
        self.possible_ports = [
            '/dev/tty.usbserial-1120'
        ]
        
        self.initUI()
        self.setup_serial()

    def setup_serial(self):
        # Try multiple ports
        for port in self.possible_ports:
            try:
                # Create serial reader thread
                self.serial_reader = SerialReader(port, 9600)
                
                # Connect signals
                self.serial_reader.data_received.connect(self.update_plot)
                self.serial_reader.error_occurred.connect(self.handle_serial_error)
                # If successful, break the loop
                print(f"Connected to serial port: {port}")
                return
            except Exception:
                continue
        
        # If no port found, show error
        QMessageBox.critical(self, "Serial Error", 
                             "Could not connect to Arduino. Please check connection.")

    def initUI(self):
        # Main window setup
        self.setWindowTitle('Arduino Serial Data Plotter')
        self.setGeometry(100, 100, 800, 600)
        self.data_points = []
        
        # Central widget and main layout
        central_widget = QWidget()
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
        # Current value label
        self.value_label = QLabel('Current Value: -')
        self.value_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.value_label)
        
        # Plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setTitle('Arduino Sensor Data')
        self.plot_widget.setLabel('left', 'Value')
        self.plot_widget.setLabel('bottom', 'Time (seconds)')
        self.plot_widget.setYRange(-0.1, 5.1, padding=0)
        main_layout.addWidget(self.plot_widget)
        
        # Create data curve
        self.curve = self.plot_widget.plot(pen='b')
        
        # Buttons layout
        button_layout = QHBoxLayout()
        
        # Start/Pause Button
        self.start_pause_btn = QPushButton('Start')
        self.start_pause_btn.clicked.connect(self.toggle_reading)
        button_layout.addWidget(self.start_pause_btn)
        
        # Stop Button
        self.stop_btn = QPushButton('Stop')
        self.stop_btn.clicked.connect(self.stop_reading)
        button_layout.addWidget(self.stop_btn)
        
        # Save Data Button
        self.save_btn = QPushButton('Save Data')
        self.save_btn.clicked.connect(self.save_data)
        button_layout.addWidget(self.save_btn)
        
        # Add button layout to main layout
        main_layout.addLayout(button_layout)

    def update_plot(self, value):
        # Update data points
        value = value*5/1023;
        self.data_points.append(value)
        
        # Limit to last 100 points to prevent memory issues
        if len(self.data_points) > 2000:
            self.data_points = self.data_points[-2000:]
        
        # Update plot
        self.curve.setData(self.data_points)
        
        # Update current value label
        self.value_label.setText(f'Current Value: {value:.2f}')

    def handle_serial_error(self, error_msg):
        # Show error message
        print(f"Serial Error: {error_msg}")
        QMessageBox.warning(self, "Serial Error", error_msg)

    def toggle_reading(self):
        if not hasattr(self, 'serial_reader') or not self.serial_reader:
            self.setup_serial()
            return
        
        if not hasattr(self, 'is_reading') or not self.is_reading:
            # Start reading
            self.start_pause_btn.setText('Pause')
            self.is_reading = True
            self.serial_reader.start()
        else:
            # Pause reading
            self.start_pause_btn.setText('Start')
            self.serial_reader.stop()
            self.is_reading = False

    def stop_reading(self):
        # Stop reading and clear data
        self.is_reading = False
        self.serial_reader.stop()
        self.start_pause_btn.setText('Start')
        
        # Clear plot and data
        self.data_points = []
        self.curve.clear()
        self.value_label.setText('Current Value: -')

    def save_data(self):
        # Pause reading if active
        if self.is_reading:
            self.toggle_reading()
        
        # If no data, do nothing
        if not self.data_points:
            return
        
        # Generate filename with timestamp
        filename = f'arduino_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        
        # Save data to CSV
        try:
            with open(filename, 'w', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(['Value'])
                for value in self.data_points:
                    csvwriter.writerow([value])
            
            print(f"Data saved to {filename}")
        except Exception as e:
            print(f"Error saving data: {e}")

    def closeEvent(self, event):
        # Close serial reader thread
        if hasattr(self, 'serial_reader'):
            self.serial_reader.stop()
        event.accept()

def main():
    app = QApplication(sys.argv)
    plotter = ArduinoDataPlotter()
    plotter.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()