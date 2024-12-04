import csv

# 입력 파일 경로와 출력 파일 경로 지정
input_file = 'IV2024.csv'   # 원본 CSV 파일
output_file = 'IV2024WithoutV.csv' # 결과를 저장할 CSV 파일

# CSV 파일 읽고 0번째, 1번째 열만 남겨 새로운 파일로 저장
with open(input_file, 'r', newline='') as csvfile:
    reader = csv.reader(csvfile)
    with open(output_file, 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        for row in reader:
            writer.writerow([row[0], row[1]])  # 0번째와 1번째 열만 선택
