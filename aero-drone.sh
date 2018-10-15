#!/bin/bash
################################################################################
# 
# Copyright (c) 2018 南京航空航天大学 航空通信网络研究室
# 
# @author   Jiang Yang (pokerpoke@qq.com)
# @date     2018-10
# @brief    
# @version  0.0.1
# 
# Last Modified:  2018-10-15
# Modified By:    Jiang Yang (pokerpoke@qq.com)
# 
################################################################################

# Run mavproxy
sudo mavproxy.py --master=/dev/ttyACM0 \
                 --out=udp:192.168.199.246:14550 \
                 --out=127.0.0.1:14550 \
                 --out=127.0.0.1:14555
                
# Select mission