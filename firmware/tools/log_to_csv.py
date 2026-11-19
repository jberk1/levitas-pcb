"""Convert a log recorded by levitas_tool.py into a CSV file.

usage: python log_to_csv.py LOGFILE OUT.csv

Every data line the firmware printed becomes one row:

    t_us        time of the sensor reading, microseconds since the Pico started
    c1..c4      raw readings of the four capacitance sensors (4095 = failed read)
    hv_V        voltage commanded on all four high voltage outputs
    height_pct  height of the mass worked out by the firmware, % of travel
                (0 = rest stop, 100 = the stop the voltage pulls it to; -1 = not calibrated)
    target_pct  height the controller is aiming for, % of travel
    mode        0 = idle, 1 = fixed voltage, 2 = calibrating, 3 = PD controller

The messages the firmware printed during the recording (calibration values,
gains, ...) and the commands that were sent are kept as comment lines
starting with '#' at the top of the file. Skip them when reading, e.g.
pandas.read_csv(path, comment='#').
"""
import sys


def main():
    log_path, csv_path = sys.argv[1], sys.argv[2]
    notes, rows = [], []
    for line in open(log_path):
        line = line.strip()
        if line.startswith('#'):
            notes.append('# firmware: ' + line[1:].strip())
        elif line.startswith('>>'):
            notes.append('# command sent: ' + line[2:].strip())
        else:
            p = line.split()
            if len(p) != 12:
                continue  # torn line
            try:
                t = int(p[0])
                counts = [int(x, 16) for x in p[1:5]]
                hv, height, target, mode = float(p[5]), float(p[9]), float(p[10]), int(p[11])
            except ValueError:
                continue
            rows.append('%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%d' % (t, *counts, hv, height, target, mode))
    with open(csv_path, 'w', newline='\n') as f:
        f.write('\n'.join(notes) + '\n')
        f.write('t_us,c1,c2,c3,c4,hv_V,height_pct,target_pct,mode\n')
        f.write('\n'.join(rows) + '\n')
    print('%s: %d rows, %d comment lines' % (csv_path, len(rows), len(notes)))


if __name__ == '__main__':
    main()
