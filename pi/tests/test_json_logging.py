import os
import json
import pytest
from unittest.mock import patch, mock_open
from logger import Logger  # Replace with your actual module name


@pytest.fixture
def logger():
    # Provide a temporary file path for testing
    file_path = "logs/test_log.json"
    # Mock the creation of directories if necessary
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    return Logger(file_path)


def test_file_creation(logger):
    # Verify that the file is opened in write mode
    with patch("builtins.open", mock_open()) as mock_file:
        Logger(logger.file_path)  # Create the logger instance
        mock_file.assert_called_once_with(logger.file_path, "w")


def test_directory_creation():
    file_path = "logs/test_log.json"
    # Ensure the directory exists before creating the logger
    with patch("builtins.open", mock_open()) as mock_file_open:
        with patch("json.dump") as mock_json_dump:
            logger = Logger(file_path)
            # Ensure directory is created (check if the directory exists before file opening)
            assert os.path.isdir(
                os.path.dirname(file_path)
            )  # Check that 'logs' directory exists
            mock_json_dump.assert_not_called()  # Ensure we haven't written yet
