import sys
import pandas as pd

def scale_third_column(input_file):


    df = pd.read_csv(input_file)

    # Invert the index
    
    df_inverted = df.iloc[::-1].reset_index(drop=True)

    # Save the new CSV file
    
    output_path = +input_file+'inverted' # Replace with your desired output file path
    df_inverted.to_csv(output_path, index=False)

    print(f"Inverted CSV file saved to {output_path}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <input_file>")
        sys.exit(1)

    input_filename = sys.argv[1]
    
    scale_third_column(input_filename)
