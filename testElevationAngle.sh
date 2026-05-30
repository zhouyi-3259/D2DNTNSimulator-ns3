#!/bin/bash

# 创建输出文件夹（如果不存在）
# mkdir -p output_results

# 循环从25到40，步长为1
for i in {35..65}; do
    echo "正在运行 minElevationAngle=$i..."
    # 假设你的程序运行命令为./waf --run，根据实际情况修改
    ./ns3 run "HO-Scheme-vs-test --minElevationAngle=$i" > output_results/2026.1.20/closest-sat-HO/vancouver_canada/minElevationAngle_$i.txt
done

echo "所有模拟已完成，结果保存在output_results文件夹中"