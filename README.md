# capacitated-vehicle-routing-problem-with-time-windows

Capacitated Vehicle Routing Problem with Time Windows (CVRPTW) solver written in Python.

Implementation is based on ["Vehicle Routing Problem with Time Windows" section in Google OR-Tools documentanion](https://developers.google.com/optimization/routing/cvrptw).


## Overview

This program solves Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).

For example, given the following network and three vehicles at the depot (node 0) to pick up all demands on all nodes in as short a time period as possible,

![network](network.png)

the program provides the solution as follows:

![route](route.png)


## Prerequisites

Python 3.6 or later is required.


## Usage

First, install the dependencies:

```shell
$ brew install graphviz --with-gts --with-librsvg --with-pango
$ pip install ortools pygraphviz
```

Alternatively, if you have `pipenv` installed, run the following command instead of the second line above:

```shell
$ pipenv install
```

Then, let's solve the sample problem!

```shell
$ ./cvrptw.py data.sample.json
```

It should output the solution as follows:

```
Route for vehicle 0:
  [Node  0: Load(0) Time( 0, 0)]
  -> [Node  9: Load(0) Time( 7, 20)]
  -> [Node 14: Load(1) Time(15, 30)]
  -> [Node 16: Load(2) Time(30, 50)]
  -> [Node 15: Load(3) Time(45, 65)]
  -> [Node 11: Load(4) Time(85, 105)]
  -> [Node  0: Load(5) Time(96, 120)]
Load of the route: 5
Time of the route: 96 min

Route for vehicle 1:
  [Node  0: Load(0) Time( 0, 3)]
  -> [Node  7: Load(0) Time( 7, 10)]
  -> [Node 13: Load(1) Time(15, 18)]
  -> [Node 12: Load(2) Time(22, 25)]
  -> [Node  4: Load(3) Time(45, 65)]
  -> [Node  3: Load(4) Time(60, 80)]
  -> [Node  1: Load(5) Time(75, 95)]
  -> [Node  0: Load(6) Time(86, 120)]
Load of the route: 6
Time of the route: 86 min

Route for vehicle 2:
  [Node  0: Load(0) Time( 0, 12)]
  -> [Node  5: Load(0) Time( 8, 20)]
  -> [Node  8: Load(1) Time(15, 30)]
  -> [Node  6: Load(2) Time(55, 75)]
  -> [Node  2: Load(3) Time(75, 86)]
  -> [Node 10: Load(4) Time(84, 95)]
  -> [Node  0: Load(5) Time(95, 120)]
Load of the route: 5
Time of the route: 95 min

Total time of all routes: 277 min
```

In addition, if you want to export images of the network and routes, use `--graph` option:

```shell
$ ./cvrptw.py --graph data.sample.json
```

It also generates `network.png` and `route.png` for vizualizing the network and the routes of vehicles.

`data.sample.json` is just sample data of a problem, so if you want to solve your own, copy `data.sample.json` and create your own `data.json`! 💪