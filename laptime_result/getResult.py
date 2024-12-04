import pandas as pd
import numpy as np
import os
import argparse

def analyze_lap_times(folder_path, output_file):
    # 결과를 저장할 리스트
    results = []
    
    # 폴더 내의 모든 CSV 파일 처리
    for filename in os.listdir(folder_path):
        if filename.endswith('.csv'):
            try:
                # CSV 파일 읽기
                file_path = os.path.join(folder_path, filename)
                df = pd.read_csv(file_path)
                
                # 1-10번째 랩타임 추출 (인덱스가 0부터 시작하므로 9까지)
                lap_times = df['lap_time'].iloc[:10]
                
                # 평균과 표준편차 계산
                avg_time = round(np.mean(lap_times), 4)
                std_time = round(np.std(lap_times),4)
                
                # 결과 저장
                results.append({
                    'file_name': filename,
                    'average_lap_time': avg_time,
                    'standard_deviation': std_time
                })
                
            except Exception as e:
                print(f"Error processing {filename}: {str(e)}")
    
    # 결과를 DataFrame으로 변환
    result_df = pd.DataFrame(results)
    
    # average_lap_time을 기준으로 오름차순 정렬
    result_df = result_df.sort_values(by='average_lap_time', ascending=True)
    
    # CSV 파일로 저장
    result_df.to_csv(output_file, index=False, float_format='%.4f')
    print(f"Analysis complete. Results saved to {output_file}")

def main():
    # 커맨드 라인 인자 파싱
    parser = argparse.ArgumentParser(description='Analyze lap times from CSV files')
    parser.add_argument('folder_path', help='Path to the folder containing CSV files')
    parser.add_argument('output_file', help='Name of the output CSV file')
    
    args = parser.parse_args()
    
    # 분석 실행
    analyze_lap_times(args.folder_path, args.output_file)

if __name__ == "__main__":
    main()