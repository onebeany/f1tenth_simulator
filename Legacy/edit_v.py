import csv
import sys

def increase_columns(file_name, constant1, constant2):
    # Read the CSV file
    with open(file_name, mode='r', newline='') as infile:
        reader = csv.reader(infile)
        rows = list(reader)

    # Modify the first and second columns
    for row in rows:
        row[0] = float(row[0]) + constant1
        row[1] = float(row[1]) + constant2

    # Write the modified data back to the same CSV file
    with open(file_name, mode='w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerows(rows)

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python script.py <file_name> <constant1> <constant2>")
        sys.exit(1)

    file_name = sys.argv[1]
    constant1 = float(sys.argv[2])
    constant2 = float(sys.argv[3])

    increase_columns(file_name, constant1, constant2)
