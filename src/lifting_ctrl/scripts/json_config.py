#!/usr/bin/env python3 
#codeing =utf-8 
# Author: RosenYin 2356838399@qq.com
# Date: 2024-01-08 14:21:44
# LastEditors: RosenYin 2356838399@qq.com
# LastEditTime: 2024-01-19 11:37:04
# FilePath: /ROS_workspace/src/BehaviorTree/lifting_ctrl/scripts/json_config.py
# Description: 
# Copyright (c) 2024 by RosenYin, All Rights Reserved. 
#/ 
import os
import json

class C_JsonConfig:
    def GetDataFromJson(self,file_name):
        # 读取当前py文件的最近目录
        current_path = os.path.dirname(__file__)

        file_path = 'config/' + file_name
        # 将刚读到的最近目录与制定的文件路径拼接，读取完整路径文件
        with open(os.path.join(current_path, file_path)) as fp:
            data = json.load(fp)
        # 返回json文件数据，存到data中并返回
        return data
    
    # 获取文件大小
    def get_file_size(self,file_name):
        return os.path.getsize(file_name)
            
    # 从json文件中获取数据
    def get_data_from_json(self,file_path):
        with open(file_path,mode = 'r', encoding='utf-8') as json_file:
              json_data = json.load(json_file)
        return None if not json_data else json_data
    
    # 将数据写入json文件
    def write_data_to_json(self,json_data):
         with open("./config/relay.json","w") as file:
                json.dump(json_data,file,indent=4)
                file.close()
