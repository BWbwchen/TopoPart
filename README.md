<div align="center">

<h3>TopoPart</h3>
<p>A topology-driven partition framework for Multi-FPGA Systems</p>
</div>

This is the implementation of paper [TopoPart: a Multi-level Topology-Driven
Partitioning Framework for Multi-FPGA
Systems](https://ieeexplore.ieee.org/document/9643481) and also my final project
for class CS516000 FPGA Architecture & CAD at NTHU.

Don't forget to check my report [here](https://drive.google.com/file/d/11EmG7Hx1afaDhlwSjeb_9S0UCgz83t59/view?usp=share_link)!

## Benchmark
I test my program in private benchmark, and the input is in [./input](./input) directory.

| Result | result | topology violation | violation rate(%) | times(s) |
|:------:|:------:|:------------------:|:-----------------:|:--------:|
|   B1   |     40 |                  0 |                 0 |     0    |
|   B2   |    163 |                 11 |               5.5 |     0    |
|   B3   |    362 |                  0 |                 0 |   0.02   |
|   B4   |    997 |                  3 |              0.15 |   0.05   |
|   B5   |  10242 |                 48 |              0.48 |   0.45   |
|   B6   |  55823 |                374 |             0.748 |   3.21   |
|   B7   | 111865 |                656 |             0.656 |   8.87   |
|   B8   | 227517 |               1362 |             0.681 |   20.8   |

<sup>
Copyright (C) 2023 Bo-Wei Chen<time.chenbw@gmail.com> - All Rights Reserved
</sup>
