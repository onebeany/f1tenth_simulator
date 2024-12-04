import sys

def process_file(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            # Split the line using ';' as the delimiter
            columns = line.strip().split(';')
            # Extract the second, third, and sixth columns (indexing starts at 0)
            required_columns = [columns[1], columns[2], columns[5]]
            # Join the required columns with ',' as the new delimiter
            new_line = ','.join(required_columns)
            # Write the new line to the output file
            outfile.write(new_line + '\n')

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py <input_file> <output_file>")
        sys.exit(1)

    input_filename = sys.argv[1]
    output_filename = sys.argv[2]
    process_file(input_filename, output_filename)
