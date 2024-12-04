import sys

def scale_third_column(input_file):
    # Read and process data
    processed_data = []
    with open(input_file, 'r') as infile:
        for line in infile:
            columns = line.strip().split(',')
            columns[2] = str(float(columns[2]) / 2)
            processed_data.append(','.join(columns))
    
    # Write processed data to the output file
    with open(input_file, 'w') as outfile:
        for line in processed_data:
            outfile.write(line + '\n')

if __name__ == "__main__":
    if len(sys.argv) != 2 : 
        print("Usage: python script.py <file_name>")
        sys.exit(1)

    input_filename = sys.argv[1]
    scale_third_column(input_filename)

