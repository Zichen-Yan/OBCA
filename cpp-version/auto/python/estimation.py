import os
import numpy as np

def extract_plan_time_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        if len(lines) >= 29:
            line = lines[28]  # 第28行索引为27
            line1 = line.split(",")[-2]
            # 提取plan_time的数值
            idx = line1.find("plan_time=")
            if idx != -1:
                s = line1[idx + len("plan_time="):].strip()
                plan_time = float(s)
            else:
                return None
            
            line2 = line.split(",")[-1]
            # 提取plan_time的数值
            idx = line2.find("avg_dis=")
            if idx != -1:
                s = line2[idx + len("avg_dis="):].strip()
                avg_dis = float(s)
            else:
                return None

            return [plan_time,avg_dis]   
    print("Fail map:", file_path)
    return None

def extract_plan_time_from_folder(folder_path):
    plan_time = []
    avg_dis = []
    for filename in os.listdir(folder_path):
        if filename.endswith(".txt"):
            file_path = os.path.join(folder_path, filename)
            res = extract_plan_time_from_file(file_path)
            if res is not None:
                plan_time.append(res[0])
                avg_dis.append(res[1])
    return plan_time,avg_dis


folder_path = "/home/byd2004/Pictures/output/1.results-opt-URE/"
plan_time,avg_dis = extract_plan_time_from_folder(folder_path)
print('success num:', len(plan_time))
print('total time:', np.sum(plan_time))
print('avg_dis:', np.mean(avg_dis))
print('\t')

folder_path = "/home/byd2004/Pictures/output/2.results-opt-safe-URE/"
plan_time,avg_dis = extract_plan_time_from_folder(folder_path)
print('success num:', len(plan_time))
print('total time:', np.sum(plan_time))
print('avg_dis:', np.mean(avg_dis))
print('\t')

folder_path = "/home/byd2004/Pictures/output/3.results-opt-safe-twoway-URE/"
plan_time,avg_dis = extract_plan_time_from_folder(folder_path)
print('success num:', len(plan_time))
print('total time:', np.sum(plan_time))
print('avg_dis:', np.mean(avg_dis))
print('\t')

folder_path = "/home/byd2004/Pictures/output/4.results-opt-twoway-ARSA/"
plan_time,avg_dis = extract_plan_time_from_folder(folder_path)
print('success num:', len(plan_time))
print('total time:', np.sum(plan_time))
print('avg_dis:', np.mean(avg_dis))
print('\t')

