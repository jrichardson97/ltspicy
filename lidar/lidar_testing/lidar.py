from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0')
# lidar = RPLidar('COM3')

info = lidar.get_info()
print(info)

for i, scan in enumerate(lidar.iter_scans()):
    print('%d: Got %d measurments' % (i, len(scan)))
    if i > 10:
        break