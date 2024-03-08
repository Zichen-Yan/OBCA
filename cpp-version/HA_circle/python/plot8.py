import matplotlib.pyplot as plt
from math import *

# WB = 2.92
# W = 1.94
# LF = 3.892
# LB = 1.093
# MAX_STEER = 0.4881
# MIN_CIRCLE = WB / tan(MAX_STEER)

WB = 2.92
W = 1.92
LF = 3.84
LB = 0.99
MAX_STEER = 0.4806
MIN_CIRCLE = WB / tan(MAX_STEER)

length = LF + LB + 0.4
width = W + 0.4
d = length / 8
radius = sqrt(pow(width / 4, 2) + pow(length / 8, 2))
center_x = length / 2 - (LB + 0.2)

Car_x = [4.0443, 4.0438, 4.043, 4.042, 4.0407, 4.0393, 4.0376, 4.0362, 4.0348, 4.0328, 4.0302, 4.0271, 4.0234, 4.0191,
         4.0144, 4.0089, 4.0011, 3.9926, 3.9835, 3.9737, 3.9633, 3.9522, 3.9405, 3.9282, 3.9277, 3.9151, 3.9013, 3.8865,
         3.8706, 3.8536, 3.8357, 3.8167, 3.7968, 3.776, 3.7856, 3.7675, 3.7486, 3.729, 3.7086, 3.6876, 3.6659, 3.6435,
         3.6206, 3.605, 3.5894, 3.5633, 3.5364, 3.5087, 3.4805, 3.4518, 3.4226, 3.393, 3.3633, 3.3333, 3.326, 3.296,
         3.2661, 3.2361, 3.2062, 3.1762, 3.1462, 3.1388, 3.1089, 3.0789, 3.0489, 3.0189, 2.9889, 2.9589, 2.9289, 2.8989,
         2.8689, 2.8492, 2.8192, 2.7892, 2.7592, 2.7292, 2.6992, 2.6692, 2.6392, 2.6092, 2.5792, 2.5493, 2.5193, 2.4893,
         2.4593, 2.4293, 2.3993, 2.3693, 2.3532, 2.3371, 2.3071, 2.2772, 2.2472, 2.2172, 2.1872, 2.1573, 2.1273, 2.0973,
         2.0673, 2.0373, 2.0073, 1.9773, 1.9473, 1.9173, 1.8873, 1.8573, 1.8273, 1.7973, 1.7673, 1.7373, 1.7073, 1.6836,
         1.6536, 1.6236, 1.5936, 1.5636, 1.5336, 1.5036, 1.4736, 1.4436, 1.4136, 1.3836, 1.3536, 1.3236, 1.2936, 1.2636,
         1.2336, 1.2036, 1.1736, 1.1436, 1.1136, 1.0836, 1.0536, 1.0236, 0.99358, 0.96358, 0.93358, 0.90359, 0.87359,
         0.84359, 0.8136, 0.7836, 0.7536, 0.72361, 0.69361, 0.66362, 0.63363, 0.60363, 0.57364, 0.59531, 0.56545,
         0.53557, 0.50569, 0.47579, 0.44588, 0.41595, 0.38602, 0.35608, 0.32613, 0.29617, 0.2662, 0.23623, 0.20625,
         0.17627, 0.14628, 0.11629, 0.08629, 0.056293, 0.026294, -0.003706, -0.033706, -0.063705, -0.093703, -0.1237,
         -0.15369, -0.18368, -0.21367, -0.24365, -0.27362, -0.29313, -0.3231, -0.35305, -0.38296, -0.41282, -0.44263,
         -0.47237, -0.48756, -0.50274, -0.53226, -0.5617, -0.59105, -0.6203, -0.64946, -0.67851, -0.69302, -0.70754,
         -0.73521, -0.76225, -0.78857, -0.81413, -0.83883, -0.86262, -0.88544, -0.90723, -0.91063, -0.93249, -0.95333,
         -0.9731, -0.99174, -1.0092, -1.0255, -1.0405, -1.0479, -1.0553, -1.0672, -1.0784, -1.089, -1.0988, -1.1081,
         -1.116, -1.124, -1.1316, -1.1389, -1.1458, -1.1522, -1.1575, -1.1624, -1.1671, -1.1716, -1.1759, -1.1801,
         -1.1824, -1.1846, -1.1874, -1.1898, -1.1919, -1.1936, -1.195, -1.1961, -1.1958, -1.1979, -1.198, -1.1982,
         -1.1967, -1.1954, -1.1941, -1.1929, -1.1926, -1.1909, -1.1887, -1.1859, -1.1826, -1.1788, -1.1744, -1.1722,
         -1.1681, -1.1635, -1.1583, -1.1525, -1.1462, -1.1392, -1.137, -1.1298, -1.122, -1.1137, -1.1048, -1.0954,
         -1.0911, -1.0815, -1.0706, -1.0585, -1.045, -1.0304, -1.0145, -0.99744, -0.99088, -0.97171, -0.95177, -0.93111,
         -0.90973, -0.88766, -0.86493, -0.84156, -0.82037, -0.7951, -0.76899, -0.74211, -0.71455, -0.68638, -0.6577,
         -0.62858, -0.59911, -0.59928, -0.57, -0.5406, -0.51107, -0.48145, -0.45173, -0.42194, -0.39863, -0.36871,
         -0.33878, -0.30884, -0.27888, -0.24892, -0.21894, -0.18896, -0.15897, -0.12898, -0.098983, -0.068984,
         -0.038985, -0.0089849, 0.017873, 0.047771, 0.077678, 0.10759, 0.13752, 0.16745, 0.19739, 0.22733, 0.25728,
         0.28724, 0.3172, 0.34717, 0.37714, 0.40712, 0.4371, 0.46709, 0.49708, 0.52707, 0.55706, 0.58706, 0.61705,
         0.64705, 0.67705, 0.70705, 0.73705, 0.76705, 0.79705, 0.82705, 0.85704, 0.88704, 0.91703, 0.94702, 0.97701,
         1.007, 1.037, 1.0669, 1.0969, 1.116, 1.146, 1.176, 1.206, 1.236, 1.266, 1.296, 1.326, 1.356, 1.386, 1.416,
         1.446, 1.476, 1.506, 1.536, 1.566, 1.596, 1.626, 1.656, 1.686, 1.716, 1.746, 1.776, 1.806, 1.836, 1.866, 1.896,
         1.926, 1.956, 1.9654, 1.9953, 2.0253, 2.0553, 2.0853, 2.1152, 2.1452, 2.1752, 2.2052, 2.2352, 2.2652, 2.2951,
         2.3251, 2.3551, 2.3851, 2.4151, 2.4451, 2.4751, 2.5051, 2.535, 2.565, 2.5906, 2.6206, 2.6506, 2.6806, 2.7106,
         2.7406, 2.7706, 2.8006, 2.8306, 2.8606, 2.8906, 2.9206, 2.9506, 2.9806, 3.0106, 3.0405, 3.0705, 3.0854, 3.1154,
         3.1453, 3.1753, 3.2053, 3.2162, 3.2462, 3.2761, 3.3061, 3.3359, 3.3657, 3.3955, 3.4251, 3.4533, 3.4822, 3.5104,
         3.538, 3.5648, 3.5908, 3.6157, 3.6395, 3.6622, 3.6836, 3.7036, 3.7084, 3.7278, 3.7472, 3.7667, 3.7863, 3.8059,
         3.8256, 3.8429, 3.8603, 3.8763, 3.8915, 3.9061, 3.9199, 3.9329, 3.9451, 3.9566, 3.9673, 3.9706, 3.9812, 3.9908,
         3.9994, 4.0069, 4.0135, 4.0189, 4.0234, 4.0268, 4.0283, 4.0308, 4.033, 4.0351, 4.037, 4.0388, 4.0403, 4.0413,
         4.0427, 4.0437, 4.0442]
Car_y = [-0.0029061, -0.032901, -0.062891, -0.092874, -0.12285, -0.15281, -0.18277, -0.2034, -0.23336, -0.2633,
         -0.29319, -0.32302, -0.35279, -0.38249, -0.41211, -0.4395, -0.46846, -0.49723, -0.52581, -0.55417, -0.5823,
         -0.61018, -0.6378, -0.66514, -0.666, -0.69321, -0.71988, -0.74594, -0.77138, -0.79613, -0.82016, -0.84342,
         -0.86588, -0.88749, -0.87434, -0.89824, -0.92155, -0.94424, -0.9663, -0.98769, -1.0084, -1.0284, -1.0477,
         -1.059, -1.0703, -1.0851, -1.0983, -1.11, -1.1201, -1.1286, -1.1355, -1.1408, -1.1444, -1.1463, -1.1463,
         -1.1481, -1.1498, -1.1514, -1.1528, -1.154, -1.1551, -1.1553, -1.1567, -1.1579, -1.159, -1.16, -1.1608,
         -1.1614, -1.162, -1.1624, -1.1626, -1.1627, -1.163, -1.1631, -1.1632, -1.1632, -1.1631, -1.1629, -1.1626,
         -1.1623, -1.1618, -1.1613, -1.1607, -1.16, -1.1592, -1.1584, -1.1575, -1.1564, -1.1558, -1.1552, -1.1535,
         -1.1519, -1.1504, -1.149, -1.1477, -1.1466, -1.1455, -1.1446, -1.1438, -1.1431, -1.1425, -1.1421, -1.1417,
         -1.1415, -1.1413, -1.1413, -1.1414, -1.1417, -1.142, -1.1424, -1.143, -1.1435, -1.1435, -1.1436, -1.1436,
         -1.1436, -1.1436, -1.1436, -1.1435, -1.1434, -1.1433, -1.1432, -1.1431, -1.1429, -1.1428, -1.1426, -1.1424,
         -1.1422, -1.1419, -1.1416, -1.1413, -1.141, -1.1407, -1.1404, -1.14, -1.1396, -1.1392, -1.1388, -1.1383,
         -1.1379, -1.1374, -1.1369, -1.1364, -1.1358, -1.1353, -1.1347, -1.1341, -1.1335, -1.1328, -1.1319, -1.1347,
         -1.1375, -1.1401, -1.1425, -1.1448, -1.147, -1.149, -1.1508, -1.1525, -1.1541, -1.1555, -1.1568, -1.1579,
         -1.1589, -1.1598, -1.1604, -1.161, -1.1614, -1.1616, -1.1617, -1.1617, -1.1615, -1.1612, -1.1607, -1.16,
         -1.1593, -1.1583, -1.1573, -1.156, -1.1552, -1.1539, -1.1521, -1.1497, -1.1469, -1.1435, -1.1396, -1.1373,
         -1.135, -1.1297, -1.1239, -1.1177, -1.111, -1.1039, -1.0965, -1.0919, -1.0874, -1.0759, -1.0629, -1.0485,
         -1.0328, -1.0158, -0.99749, -0.97802, -0.9574, -0.95311, -0.93257, -0.91099, -0.88843, -0.86493, -0.84055,
         -0.81536, -0.7894, -0.77485, -0.7603, -0.73275, -0.70493, -0.67685, -0.64852, -0.61998, -0.59337, -0.56446,
         -0.53545, -0.50634, -0.47714, -0.44784, -0.42181, -0.39221, -0.36258, -0.33293, -0.30324, -0.27353, -0.25601,
         -0.2385, -0.20862, -0.17872, -0.14879, -0.11884, -0.088875, -0.058896, -0.053784, -0.023857, -0.0072265,
         0.0094037, 0.03937, 0.06934, 0.099313, 0.12929, 0.13662, 0.16658, 0.19649, 0.22637, 0.25618, 0.28594, 0.31562,
         0.32873, 0.35846, 0.3881, 0.41765, 0.44708, 0.4764, 0.50558, 0.51427, 0.54339, 0.57237, 0.60119, 0.62985,
         0.65833, 0.67032, 0.69875, 0.7267, 0.75412, 0.78094, 0.8071, 0.83256, 0.85724, 0.86588, 0.88895, 0.91137,
         0.93311, 0.95415, 0.97448, 0.99405, 1.0129, 1.0286, 1.0448, 1.0596, 1.0729, 1.0847, 1.095, 1.1038, 1.111,
         1.1166, 1.1158, 1.1224, 1.1283, 1.1336, 1.1384, 1.1425, 1.146, 1.1482, 1.1505, 1.1525, 1.1543, 1.1559, 1.1574,
         1.1586, 1.1597, 1.1605, 1.1612, 1.1616, 1.1619, 1.162, 1.1618, 1.161, 1.1585, 1.1562, 1.1539, 1.1518, 1.1498,
         1.1478, 1.146, 1.1443, 1.1427, 1.1412, 1.1398, 1.1385, 1.1374, 1.1363, 1.1354, 1.1345, 1.1338, 1.1332, 1.1327,
         1.1323, 1.132, 1.1318, 1.1317, 1.1317, 1.1319, 1.1321, 1.1325, 1.133, 1.1335, 1.1342, 1.135, 1.1359, 1.1369,
         1.138, 1.1393, 1.1406, 1.1414, 1.1416, 1.1419, 1.1421, 1.1424, 1.1426, 1.1427, 1.1429, 1.1431, 1.1432, 1.1433,
         1.1434, 1.1435, 1.1435, 1.1436, 1.1436, 1.1436, 1.1436, 1.1435, 1.1435, 1.1434, 1.1433, 1.1432, 1.1431, 1.1429,
         1.1428, 1.1426, 1.1424, 1.1421, 1.1418, 1.1431, 1.1443, 1.1455, 1.1466, 1.1478, 1.1489, 1.15, 1.151, 1.152,
         1.153, 1.154, 1.1549, 1.1558, 1.1567, 1.1576, 1.1584, 1.1592, 1.16, 1.1607, 1.1614, 1.162, 1.1624, 1.1627,
         1.163, 1.1631, 1.1632, 1.1632, 1.1631, 1.1629, 1.1626, 1.1622, 1.1618, 1.1612, 1.1606, 1.1599, 1.159, 1.1581,
         1.1577, 1.1563, 1.1551, 1.1538, 1.1527, 1.1522, 1.1515, 1.1501, 1.148, 1.1454, 1.1421, 1.1381, 1.1336, 1.1282,
         1.1199, 1.1099, 1.0981, 1.0847, 1.0696, 1.0529, 1.0347, 1.015, 0.994, 0.97168, 0.96581, 0.94289, 0.92003,
         0.89722, 0.87448, 0.85179, 0.82916, 0.8078, 0.78644, 0.76105, 0.73523, 0.70898, 0.68233, 0.65531, 0.62793,
         0.60021, 0.57218, 0.5626, 0.53454, 0.50612, 0.47738, 0.44835, 0.41907, 0.38958, 0.35991, 0.3301, 0.31096,
         0.28106, 0.25114, 0.22121, 0.19128, 0.16132, 0.13136, 0.11028, 0.080315, 0.050331, 0.020336]

Car_x10 = [4.0443, 4.0362, 4.0089, 3.9277, 3.776, 3.7856, 3.6875, 3.5894, 3.3333, 3.326, 3.1388, 2.8689, 2.8492, 2.5792,
           2.3371, 2.0673, 1.7973, 1.6836, 1.4136, 1.1436, 0.87359, 0.60363, 0.59531, 0.32613, 0.056293, -0.21367,
           -0.29313, -0.50274, -0.70754, -0.91063, -1.0553, -1.116, -1.1575, -1.1846, -1.1958, -1.1982, -1.1926,
           -1.1722, -1.137, -1.0911, -0.99088, -0.82037, -0.59928, -0.39863, -0.12898, 0.017873, 0.28724, 0.55706,
           0.82705, 1.0969, 1.116, 1.386, 1.656, 1.926, 1.9654, 2.2352, 2.5051, 2.5906, 2.8606, 3.0854, 3.2162, 3.4533,
           3.6836, 3.7084, 3.8603, 3.9706, 4.0283, 4.0413]
Car_y10 = [-0.0029061, -0.2034, -0.4395, -0.666, -0.88749, -0.87434, -0.97234, -1.0703, -1.1463, -1.1463, -1.1553,
           -1.1626, -1.1627, -1.1618, -1.1552, -1.1438, -1.1417, -1.1435, -1.1433, -1.1416, -1.1383, -1.1335, -1.1319,
           -1.1525, -1.1614, -1.1583, -1.1552, -1.135, -1.0874, -0.95311, -0.7603, -0.59337, -0.42181, -0.2385,
           -0.053784, 0.0094037, 0.13662, 0.32873, 0.51427, 0.67032, 0.86588, 1.0286, 1.1158, 1.1482, 1.1612, 1.161,
           1.1427, 1.1332, 1.1325, 1.1406, 1.1414, 1.1432, 1.1435, 1.1424, 1.1418, 1.152, 1.16, 1.162, 1.1626, 1.1577,
           1.1522, 1.1282, 0.994, 0.96581, 0.78644, 0.5626, 0.31096, 0.11028]

# 定义参数
tmp1 = (center_x + 3 * d - 0.2, width / 4 - 0.28)
tmp2 = (center_x + d, width / 4 - 0.15)
tmp3 = (center_x - d, width / 4 - 0.15)
tmp4 = (center_x - 3 * d + 0.2, width / 4 - 0.28)

tmp5 = (center_x + 3 * d - 0.2, -width / 4 + 0.28)
tmp6 = (center_x + d, -width / 4 + 0.15)
tmp7 = (center_x - d, -width / 4 + 0.15)
tmp8 = (center_x - 3 * d + 0.2, -width / 4 + 0.28)

# 创建图像
fig, ax = plt.subplots()

plt.scatter(Car_x, Car_y, color='blue', marker='.', label='Data Points', s=1)
plt.scatter(Car_x10, Car_y10, color='yellow', marker='.', label='Data Points', s=1)

# 绘制圆
circle_center1 = (tmp1[0], tmp1[1])
circle_center2 = (tmp2[0], tmp2[1])
circle_center3 = (tmp3[0], tmp3[1])
circle_center4 = (tmp4[0], tmp4[1])

circle_center5 = (tmp5[0], tmp5[1])
circle_center6 = (tmp6[0], tmp6[1])
circle_center7 = (tmp7[0], tmp7[1])
circle_center8 = (tmp8[0], tmp8[1])

circle1 = plt.Circle(circle_center1, radius + 0.1, color='r', fill=False, linewidth=2)
circle2 = plt.Circle(circle_center2, radius, color='r', fill=False, linewidth=2)
circle3 = plt.Circle(circle_center3, radius, color='r', fill=False, linewidth=2)
circle4 = plt.Circle(circle_center4, radius + 0.1, color='r', fill=False, linewidth=2)
circle5 = plt.Circle(circle_center5, radius + 0.1, color='r', fill=False, linewidth=2)
circle6 = plt.Circle(circle_center6, radius, color='r', fill=False, linewidth=2)
circle7 = plt.Circle(circle_center7, radius, color='r', fill=False, linewidth=2)
circle8 = plt.Circle(circle_center8, radius + 0.1, color='r', fill=False, linewidth=2)

ax.add_patch(circle1)
ax.add_patch(circle2)
ax.add_patch(circle3)
ax.add_patch(circle4)
ax.add_patch(circle5)
ax.add_patch(circle6)
ax.add_patch(circle7)
ax.add_patch(circle8)

# 显示图像
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Rectangle and Circles')
plt.grid(True)
plt.axis('equal')
plt.show()
