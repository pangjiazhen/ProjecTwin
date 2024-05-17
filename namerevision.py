import os
import shutil
import pandas as pd
import re
# 读取 Excel 文件，指定表格名称和列名称
df = pd.read_excel('C:\\Users\\Lenovo\\Desktop\\2024会议\\csie.xlsx', usecols=['姓名'])
df2 = pd.read_excel('C:\\Users\\Lenovo\\Desktop\\2024会议\\csie.xlsx', usecols=['id'])

# 获取列数据
column_data = df['姓名'].tolist()
ID_data = df2['id'].tolist()

for i in range(len(column_data)):
    column_data[i] = column_data[i].split(' (')[0]
#
# # 打印列数据
for item in column_data:
    print(item)
# # 指定文件夹路径
# folder_path = 'C:\\Users\\Lenovo\\Desktop\\2024会议\\会议邀请函\\群发邮件'
# aaa= ['']
# file_extension = '.pdf'  # 文件扩展名
# file_list = os.listdir(folder_path)
# # 遍历文件夹中的所有文件
# for i, filename in enumerate(file_list):
#     new_filename = item[i] + file_extension  # 从列表中获取新的文件名
#     old_filepath = os.path.join(folder_path, filename)
#     new_filepath = os.path.join(folder_path, new_filename)
#     os.rename(old_filepath, new_filepath)

folder_path = 'C:\\Users\\Lenovo\\Desktop\\2024会议\\会议邀请函\\csie\\csie群发'  # 设置文件夹路径
file_extension = '.pdf'  # 文件扩展名
name_list = column_data

# 获取文件夹中所有符合条件的文件
file_list = [f for f in os.listdir(folder_path) if f.endswith(file_extension)]

# 根据文件名排序文件列表
file_list.sort(key=lambda x: os.path.getmtime(os.path.join(folder_path, x)))

# 遍历文件列表，重命名文件
for i, filename in enumerate(file_list):
    new_filename = 'paper ' + str(ID_data[i]) +' ' + 'invitation letter '+ re.sub(r'[\\/:"*?<>|]', '', name_list[i]) + file_extension  # 替换特殊字符
    old_filepath = os.path.join(folder_path, filename)
    new_filepath = os.path.join(folder_path, new_filename)
    os.rename(old_filepath, new_filepath)