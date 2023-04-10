from gurobipy import *

# 节点类，储存流入与流出该节点的弧集
class Node():
    def __init__(self, sta, t):
        self.sta_located = sta
        self.t_located = t
        self.in_arcs = {}  # 流入该节点的弧集，以trainNo为key，弧为value
        self.out_arcs = {}  # 流出该节点的弧集，以trainNo为key, 弧为value
        self.trainOccupancyVar = {} # 某车是否占用该节点，1为占用，0为不占用
        self.occupancyVar = None
    def __repr__(self):
        return "Node: " + str(self.sta_located) + " at time " + str(self.t_located)

    def __str__(self):
        return "Node: sta_" + self.sta_located + ";" + "t_" + self.t_located

    def set_train_occupancy(self, trainList, model):
        '''
        set individual train occupancy at this node according to the specific train's inflows at this node
        :return:
        '''
        for train in trainList:
            if train.traNo not in self.in_arcs.keys() or train.traNo not in self.out_arcs.keys(): # 如果该车不走该节点
                continue
            self.trainOccupancyVar[train.traNo] = model.addVar(vtype=GRB.BINARY,
                                                               name="ocup_" + str(self.sta_located) + '_' + str(self.t_located) + '_' + train.traNo)
            lhs = LinExpr(0)
            lhs.addTerms(1, self.trainOccupancyVar[train.traNo])
            for in_arcs in self.in_arcs[train.traNo]:
                lhs.addTerms(-1, in_arcs)
            model.addConstr(lhs == 0) # 节点占用变量等于流入该节点的所有arc之和

    def set_occupancy(self, trainList, model):
        '''
        set node occupancy, equals the sum of train occupancy at this nodes
        :param model:
        :return:
        '''
        self.occupancyVar = model.addVar(vtype=GRB.BINARY,
                                         name="ocup_" + str(self.sta_located) + '_' + str(
                                          self.t_located))
        lhs = LinExpr(0)
        lhs.addTerms(1, self.occupancyVar)

        for train in trainList:
            if train.traNo in self.trainOccupancyVar.keys():
                lhs.addTerms(-1, self.trainOccupancyVar[train.traNo])
        model.addConstr(lhs == 0)

    def associate_with_incoming_arcs(self, train):
        '''
        associate node with train arcs, add incoming arcs to nodes
        :param train:
        :return:
        '''
        sta_node = self.sta_located
        t_node = self.t_located

        if sta_node not in train.v_staList: # 若该车不经过该站，直接退出
            return -1

        # associate incoming arcs
        # train arc structure: key：[dep, arr], value为弧集字典(key: [t], value: arc字典, key为arc_length)
        if sta_node != train.v_staList[0]:  # 不为第一站，则拥有上一站
            preSta = train.v_staList[train.v_staList.index(sta_node) - 1]  # 已经考虑列车停站情况的车站集
            curSta = sta_node
            cur_arcs = train.arcs[preSta, curSta] # 这个区间/车站的所有弧
            for start_t in cur_arcs.keys():
                arcs_from_start_t = cur_arcs[start_t] #从上一节点在start_t伸出来的arc，包括区间的一个arc和停站弧的若干个arc
                for arc_length, arc_var in arcs_from_start_t.items():
                    if (arc_length + start_t == t_node):  # 若该弧流入该节点
                        # 若不包含该车的弧列表，则先生成弧列表
                        if train.traNo not in self.in_arcs.keys():
                            self.in_arcs[train.traNo] = []
                        self.in_arcs[train.traNo].append(arc_var)


    def associate_with_outgoing_arcs(self, train):
        '''
        associate node with train arcs, add outgoing arcs to nodes
        :param train:
        :return:
        '''
        sta_node = self.sta_located
        t_node = self.t_located

        if sta_node not in train.v_staList:
            return -1

        # associate outgoing arcs
        if sta_node != train.v_staList[-1]: #不为最后一站, 则拥有下一站
            curSta = sta_node
            nextSta = train.v_staList[train.v_staList.index(sta_node) + 1]
            cur_arcs = train.arcs[curSta, nextSta] # cur_arcs以t为key，获取各个length的arc
            # 由于此时判断该点是不是一些arc的出发点，则直接看t在不在arcs的key里，若在，则将arcs里这个t的所有arc放进来
            if t_node in cur_arcs.keys(): # 如果点t在列车区间/车站弧集当中
                self.out_arcs[train.traNo] = []
                for arc_var in cur_arcs[t_node].values():
                    self.out_arcs[train.traNo].append(arc_var)