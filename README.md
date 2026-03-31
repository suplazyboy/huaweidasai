# 选手运行demo命令

## 命令格式
```bash
bash run_and_test.sh <solution_file> <input_file> <output_file>
```
其中:
1. solution_file：示例程序，位于demos目录下，有c、c++、java和python四种语言  
2. input_file: 用例数据, 位于data目录下，有多组测试用例，命名格式为：practice_*.in  
3. output_file: 示例程序输出结果文件


## 执行如下命令可运行此demo程序
```
bash run_and_test.sh ./demos/Solution.py ./data/practice_3.in ./PythonOut/practice_3.out
bash run_and_test.sh ./demos/Solution_WY.cpp ./data/practice_3.in ./PythonOut/practice_3.out
bash run_and_test.sh ./C++Solution/Solution.cpp ./data/practice_1.in ./C++Out/practice_1.out
bash run_and_test.sh ./C++Solution/Solution.cpp ./data/practice_2.in ./C++Out/practice_2.out
bash run_and_test.sh ./C++Solution/Solution.cpp ./data/practice_3.in ./C++Out/practice_3.out
bash run_and_test.sh ./C++Solution/Solution.cpp ./data/practice_4.in ./C++Out/practice_4.out
bash run_and_test.sh ./C++Solution/Solution.cpp ./data/practice_5.in ./C++Out/practice_5.out
bash run_and_test.sh ./C++Solution/Solution.cpp ./data/practice_6.in ./C++Out/practice_6.out
bash run_and_test.sh ./C++Solution/Solution.cpp ./data/practice_7.in ./C++Out/practice_7.out
```

