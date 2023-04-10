import regex

from Train import *
from gurobipy import *
from Node import *
import matplotlib.pyplot as plt
import numpy as np
import csv

model = Model('TTP_TimeSpaceNetwork')
TimeSpan = 160
H_dep = 5
H_arr = 5
'''
initialize stations, sections, trains and train arcs
'''
staList = []  # 实际车站列表
v_staList = []  # 时空网车站列表，车站一分为二 # 源节点s, 终结点t
secTimes_300 = {}  # total miles for stations
secTimes_350 = {}  # total miles for stations
miles = []
trainList = []
nodeList = {}  # 先用车站做key，再用t做key索引到node


def readStation(path):
    f = open(path, 'r')
    lines = f.readlines()
    count = 0
    for line in lines:
        if count == 0:
            count += 1
            continue

        line = line[:-1]
        str = re.split(r",", line)
        staList.append(str[0])
        miles.append(int(str[1]))
    for sta in staList:
        if staList.index(sta) != 0:  # 不为首站，有到达
            v_staList.append('_' + sta)
        if staList.index(sta) != len(staList) - 1:
            v_staList.append(sta + '_')  # 不为尾站，又出发


def readSection(path):
    f = open(path, 'r')
    lines = f.readlines()
    count = 0
    for line in lines:
        if count == 0:
            count += 1
            continue
        line = line[:-1]
        str = re.split(r",", line)
        pair = re.split(r"-", str[0])
        secTimes_300[pair[0], pair[1]] = int(str[1])
        secTimes_350[pair[0], pair[1]] = int(str[2])


def readTrain(path):
    f = open(path, 'r')
    lines = f.readlines()
    count = 0
    for line in lines:
        if count == 0:
            count += 1
            continue
        line = line[:-1]
        str = re.split(r",", line)
        train = Train(str[0], 0, TimeSpan)
        train.speed = str[1]
        for i in range(0, len(staList)):
            if (str[i + 2] == '1'):
                train.linePlan[staList[i]] = 1
            else:
                train.linePlan[staList[i]] = 0
        train.init_traStaList(staList)
        if train.speed == '350':
            train.create_arcs_GRB(secTimes_350, model, TimeSpan)
        else:
            train.create_arcs_GRB(secTimes_300, model, TimeSpan)
        trainList.append(train)


readStation('data/station.csv')
readSection('data/section.csv')
readTrain('data/train.csv')


def init_nodes():
    '''
    initialize nodes, associated with incoming nad outgoing train arcs
    '''
    # initialize node dictionary with key [sta][t]
    for sta in v_staList:  # 循环车站
        nodeList[sta] = {}
        for t in range(0, TimeSpan):  # 循环时刻t
            node = Node(sta, t)
            nodeList[sta][t] = node


# associate node with train arcs, add incoming and outgoing arcs to nodes
def associate_arcs(trainList):
    for nodes_sta in nodeList.values():
        for node in nodes_sta.values():
            for train in trainList:
                node.associate_with_outgoing_arcs(train)
                node.associate_with_incoming_arcs(train)


'''
complementary methods 辅助方法,生成车站到达与出发的key
'''
def get_arrSta(arr):
    return '_' + arr


def get_depSta(dep):
    return dep + '_'


'''
set flow balance constraints
'''
source_node = 's_'
sink_node = '_t'


def set_minimize_travel_time_Obj(model):
    obj = LinExpr()
    for train in trainList:
        # dep-arr => t => span
        for i in range(1, len(train.v_staList) - 1):
            curSta = train.v_staList[i]
            nextSta = train.v_staList[i + 1]
            for t, arcs_t in train.arcs[curSta, nextSta].items():
                for arc_length, arc_var in arcs_t.items():
                    obj.addTerms(arc_length, arc_var)
    model.setObjective(obj, GRB.MINIMIZE)


def set_source_flow_Constr(model):
    # source node arc => only one arc exists, source dep: 's_'
    for train in trainList:
        lhs = LinExpr(0)
        for t_arcs in train.arcs[source_node, get_depSta(train.depSta)].values():
            # default： only possess one arc(key: 0)
            lhs.addTerms(1, t_arcs[0])
        model.addConstr(lhs == 1)


def set_sink_flow_Constr(model):
    # sink node arc => only one arc exists, sink arr: '_t'
    for train in trainList:
        lhs = LinExpr(0)
        for t_arcs in train.arcs[get_arrSta(train.arrSta), '_t'].values():
            # default： only possess one arc(key: 0), note: _E, _t
            lhs.addTerms(1, t_arcs[0])
        model.addConstr(lhs == 1)


def set_flow_balance_Constr(model):
    # flow balance during actual stations
    # for a specific train at any node it passes, flow balance must be kept
    for train in trainList:
        for i in range(1, len(train.v_staList) - 1):  # 去掉虚拟节点
            sta = train.v_staList[i]
            for t in range(0, TimeSpan):
                node = nodeList[sta][t]
                lhs = LinExpr(0)  # 对每一个车在每一个time space node都有一个约束，流入等于流出
                if train.traNo in node.in_arcs.keys() and train.traNo in node.out_arcs.keys():  # 若该train有同时流入和流出该节点的弧
                    for arc in node.in_arcs[train.traNo]:
                        lhs.addTerms(1, arc)
                    for arc in node.out_arcs[train.traNo]:
                        lhs.addTerms(-1, arc)
                    model.addConstr(lhs == 0)


def set_node_occupancy_variables(model):
    for sta, nodes in nodeList.items():
        for node in nodes.values():
            node.set_train_occupancy(trainList, model)  # set specific train occupancy at node, arc related
            node.set_occupancy(trainList, model)  # set node occupancy, all trains related


def set_headway_constraint(model):
    '''
    for each departure node in graph
    no successive occupancies for nodes within the time interval of the departure headway
    :return:
    '''
    for sta in v_staList:
        for t in range(0, TimeSpan):
            H = 0
            if sta.endswith('_'):
                H = H_dep
            elif sta.startswith('_'):
                H = H_arr
            if t + H - 1 >= TimeSpan:
                break
            lhs = LinExpr(0)
            for t_range in range(0, H):
                node = nodeList[sta][t + t_range]
                if node.occupancyVar is not None:
                    lhs.addTerms(1, node.occupancyVar)
            model.addConstr(lhs <= 1)


def get_train_paths_from_result():
    for train in trainList:
        print("===============Tra_" + train.traNo + "======================")
        for i in range(len(train.v_staList) - 1):
            curSta = train.v_staList[i]
            nextSta = train.v_staList[i + 1]
            for t, arcs_t in train.arcs[curSta, nextSta].items():
                for arc_length, arc_var in arcs_t.items():
                    if (arc_var.x == 1):
                        print(curSta + "(" + str(t) + ") => " + nextSta + "(" + str(t + arc_length) + ")")
                        train.timetable[curSta] = t
                        train.timetable[nextSta] = t + arc_length
    for sta in v_staList:
        for t in range(0, TimeSpan):
            if nodeList[sta][t].occupancyVar.x == 1:
                print(sta + "-" + str(t) + " is occupied")


'''
initialization
'''
# init_trains()
init_nodes()
associate_arcs(trainList=trainList)

'''
flow balance constraints
'''
set_source_flow_Constr(model)
set_sink_flow_Constr(model)
set_flow_balance_Constr(model)

'''
variable transitions
'''
set_node_occupancy_variables(model)

'''
track capacity constraint
'''
set_headway_constraint(model)

'''
objective function
'''
set_minimize_travel_time_Obj(model)

'''
solve and get result
'''
model.optimize()
model.write('model.lp')
get_train_paths_from_result()

'''
draw timetable
'''
fig = plt.figure(figsize=(7, 7), dpi=200)
color_value = {
    '0': 'midnightblue',
    '1': 'mediumblue',
    '2': 'c',
    '3': 'orangered',
    '4': 'm',
    '5': 'fuchsia',
    '6': 'olive'
}

xlist = []
ylist = []
for i in range(len(trainList)):
    train = trainList[i]
    xlist = []
    ylist = []
    for sta_id in range(len(train.staList)):
        sta = train.staList[sta_id]
        if sta_id != 0:  # 不为首站, 有到达
            if "_" + sta in train.v_staList:
                xlist.append(train.timetable["_" + sta])
                ylist.append(miles[staList.index(sta)])
        if sta_id != len(train.staList) - 1:  # 不为末站，有出发
            if sta + "_" in train.v_staList:
                xlist.append(train.timetable[sta + "_"])
                ylist.append(miles[staList.index(sta)])
    plt.plot(xlist, ylist, color=color_value[str(i % 7)], linewidth=1.5)
    plt.text(xlist[0] + 0.8, ylist[0] + 4, train.traNo, ha='center', va='bottom',
             color=color_value[str(i % 7)], weight='bold', family='Times new roman', fontsize=9)

plt.grid(True)  # show the grid
plt.ylim(0, miles[-1])  # y range

plt.xlim(0, TimeSpan)  # x range
plt.xticks(np.linspace(0, TimeSpan, int(TimeSpan / 10 + 1)))

plt.yticks(miles, staList, family='Times new roman')
plt.xlabel('Time (min)', family='Times new roman')
plt.ylabel('Space (km)', family='Times new roman')
plt.savefig('Gurobi_LP.pdf', bbox_inches = 'tight')
plt.show()
