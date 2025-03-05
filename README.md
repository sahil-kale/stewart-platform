# 3-DOF Stewart Platform

## Control Architecture
![image](https://github.com/user-attachments/assets/f244df79-96c0-45f6-99fd-0ee2d71eee96)
The full system report is available here: [MTE-380-Final-Report.pdf](https://github.com/user-attachments/files/18014176/MTE-380-Final-Report.pdf)


## Platform 

![image](https://github.com/user-attachments/assets/83f0e549-1c1d-4a22-9210-8dc8db567784)

## Developer Workflow
0. First time setup: run `python3 scripts/setup.py`
1. Run `python3 scripts/test_pi_code.py` to run the python unit tests
2. Run `python3 scripts/format.py` to run the formatter
3. Run `python3 scripts/build_arduino.py --deploy` to deploy the code to the arduino

## Visualization Command
`python3 pi/main.py --visualize --virtual --inhibit_controller`