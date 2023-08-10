//
// Created by yaozhuo on 2021/9/3.
//

#ifndef _COLOR_TABLE_H_
#define _COLOR_TABLE_H_

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc.hpp"//draw a line
#include "opencv2/opencv.hpp"

    const cv::Vec3b COLOR_TABLE[30] = {
            cv::Vec3b(147, 20, 255), cv::Vec3b(0, 165, 255),   //  0深粉色             1橙色
            cv::Vec3b(124, 252, 0), cv::Vec3b(255, 0, 0),     //  2草坪绿             3纯蓝
            cv::Vec3b(238, 238, 175), cv::Vec3b(255, 255, 0),   //  4苍白的绿宝石       5青色
            cv::Vec3b(160, 158, 95), cv::Vec3b(155, 155, 125), //  6军校蓝             7淡青色
            cv::Vec3b(238, 130, 238), cv::Vec3b(0, 215, 255),   //  8紫罗兰             9金
            cv::Vec3b(209, 206, 0), cv::Vec3b(147, 112, 219), //  10深绿宝石          11苍白的紫罗兰红色
            cv::Vec3b(222, 196, 176), cv::Vec3b(255, 144, 30),  //  12淡钢蓝            13道奇蓝
            cv::Vec3b(143, 188, 143), cv::Vec3b(2, 248, 255),   //  14深海洋绿          15玉米色
            cv::Vec3b(216, 191, 216), cv::Vec3b(221, 160, 221), //  16蓟                17李子
            cv::Vec3b(180, 105, 255), cv::Vec3b(193, 182, 255), //  18热情的粉红        19浅粉色
            cv::Vec3b(112, 25, 25), cv::Vec3b(170, 178, 32),  //  20午夜的蓝色        21浅海洋绿
            cv::Vec3b(133, 21, 199), cv::Vec3b(214, 112, 218), //  22适中的紫罗兰红色  23兰花的紫色
            cv::Vec3b(225, 105, 65), cv::Vec3b(237, 149, 100), //  24皇军蓝            25矢车菊的蓝色
            cv::Vec3b(255, 248, 240), cv::Vec3b(235, 206, 135), //  26爱丽丝蓝          27天蓝色
            cv::Vec3b(208, 224, 64), cv::Vec3b(113, 179, 60)   //  28绿宝石            29春天的绿色
    };

#endif //_COLOR_TABLE_H
