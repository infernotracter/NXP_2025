#########################################################################
# File Name: m.sh
# Author: zzc18007239316@163.com
# mail: zzc18007239316@163.com
# Created Time: Wed Feb 19 18:09:27 2025
#########################################################################
#!/bin/bash

# 拉取最新代码
git pull origin master

# 提示用户输入提交注释
echo "请输入本次推送的提交注释："
read commit_message

# 添加所有更改的文件
git add .

# 提交更改
git commit -m "$commit_message"

# 推送更改到远程仓库
git push origin master
