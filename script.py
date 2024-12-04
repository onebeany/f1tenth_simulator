import csv
import sys

def find_minmax_third_column(csv_file):
    """
    CSV 파일에서 세 번째 열의 최소값과 최대값을 찾습니다.
    
    Args:
        csv_file (str): CSV 파일의 경로
    
    Returns:
        tuple: (최소값, 최대값)
    """
    try:
        # CSV 파일 읽기
        with open(csv_file, 'r') as f:
            csv_reader = csv.reader(f)
            # 세 번째 열의 값들을 float으로 변환하여 리스트로 만듦
            third_column = [float(row[2]) for row in csv_reader]
            
        if not third_column:
            print("Error: CSV 파일이 비어있거나 세 번째 열이 없습니다.")
            return None
            
        # 최소값과 최대값 찾기
        min_value = min(third_column)
        max_value = max(third_column)
        
        return min_value, max_value
        
    except FileNotFoundError:
        print(f"Error: '{csv_file}' 파일을 찾을 수 없습니다.")
    except IndexError:
        print("Error: CSV 파일에 세 번째 열이 없습니다.")
    except ValueError:
        print("Error: 세 번째 열에 숫자가 아닌 값이 포함되어 있습니다.")
    except Exception as e:
        print(f"Error: 예상치 못한 오류가 발생했습니다 - {str(e)}")
    
    return None

def main():
    # 명령행 인자 확인
    if len(sys.argv) != 2:
        print("Usage: python script.py <csv_file>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    result = find_minmax_third_column(csv_file)
    
    if result:
        min_val, max_val = result
        print(f"세 번째 열의 최소값: {min_val}")
        print(f"세 번째 열의 최대값: {max_val}")

if __name__ == "__main__":
    main()