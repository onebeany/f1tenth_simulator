import sys

def scale_third_column(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            # Split the line using ',' as the delimiter
            columns = line.strip().split(',')
            # Scale the third column by multiplying its value by 2
            columns[2] = str(float(columns[2]) * 2)
            # Join the columns back into a single string with ',' as the delimiter
            new_line = ','.join(columns)
            # Write the new line to the output file
            outfile.write(new_line + '\n')

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py <input_file> <output_file>")
        sys.exit(1)

    input_filename = sys.argv[1]
    output_filename = sys.argv[2]
    scale_third_column(input_filename, output_filename)
