import pandas as pd

def process_csv(file_path, output_path):
    # 데이터 불러오기
    df = pd.read_csv(file_path, delimiter=',', header=None)
    
    # 모든 열에 대해 소수점 조정
    for col in df.columns:
        if df[col].dtype == float:
            df[col] = df[col].round(6)
    
    # 수정된 데이터를 새 파일로 저장
    df.to_csv(output_path, sep=',', index=False, header=None)

# 사용 예
file_path = '/home/onebean/sim_ws/src/racelines/breath_m0.5_v1.0.csv'
output_path = '/home/onebean/sim_ws/src/racelines/breath_m0.5_v1.0.csv'
process_csv(file_path, output_path)

