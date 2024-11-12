from unittest.mock import mock_open, patch
import json
import time
from logger import Logger  # Replace with the actual import if needed

def test_log_new_data():
    # Initialize logger with a dummy file path
    file_path = "logs/test_log.json"
    logger = Logger(file_path)
    
    # Create a sample data structure to log
    data = [
        {
            "time": 0.0,
            "camera_measured_x": 5.1,
            "camera_measured_y": -3.4,
            "camera_filtered_x": 2.1,
            "camera_filtered_y": -1.7,
            "desired_position_x": 0.0,
            "desired_position_y": 0.0,
            "pitch_rad": 0.2,
            "roll_rad": -0.1,
            "height": 100.0,
            "servo_1_angle": 45,
            "servo_2_angle": 30,
            "servo_3_angle": 60
        }
    ]

    # Mock file opening and json dumping
    with patch("builtins.open", mock_open()) as mock_file_open:
        with patch("json.dump") as mock_json_dump:
            # Call the logging function
            logger.log_new_data(data)
            
            # Verify the file was opened in write mode
            mock_file_open.assert_called_once_with(file_path, "w")

            # Check that json.dump was called with the correct data and file object
            mock_json_dump.assert_called_once_with(data, mock_file_open(), indent=4)
