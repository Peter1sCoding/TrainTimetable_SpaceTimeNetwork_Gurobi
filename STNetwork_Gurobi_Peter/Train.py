# Here defines the info about the trains
from gurobipy import *


class Train():
    def __init__(self, traNo, dep_LB, dep_UB):
        '''
        construct
        :param traNo:
        :param dep_LB:
        :param dep_UB:
        '''
        self.traNo = traNo  # 列车车次
        self.dep_LB = dep_LB  # 始发时间窗下界
        self.dep_UB = dep_UB  # 始发时间窗上界
        self.arcs = {}  # 内含弧两个边界点的key：[dep, arr], value为弧集字典(key: [t], value: arc字典, key为arc_length) 三层字典嵌套: dep-arr => t => span
        # self.stop_addTime = 3  # 停车附加时分
        # self.start_addTime = 2  # 起车附加时分
        self.stop_addTime = 0  # 停车附加时分
        self.start_addTime = 0  # 起车附加时分
        self.min_dwellTime = 2  # 最小停站时分
        self.max_dwellTime = 15  # 最大停站时分
        self.depSta = None
        self.arrSta = None
        self.v_staList = [] # dual stations
        self.staList = [] # actual stations
        self.linePlan = {}  # 开行方案字典
        self.nodeOccupancy = {} # 是否占用某节点，key为sta => t
        self.timetable = {} # 以virtual station为key，存int值
        self.speed = None # 列车速度，300,350

    def init_traStaList(self, allStaList):
        '''
        create train staList, include s_, _t， only contains nodes associated with this train
        :param allStaList:
        :return:
        '''
        for sta in allStaList:
            if sta in self.linePlan.keys():
                self.staList.append(sta)
        self.v_staList.append('s_')
        for i in range(len(self.staList)):
            if i != 0:
                self.v_staList.append('_' + self.staList[i])
            if i != len(self.staList) - 1:  # 若不为实际车站的最后一站，则加上sta_
                self.v_staList.append(self.staList[i] + '_')
        self.v_staList.append('_t')

    def create_arcs_GRB(self, secTimes, model, TimeSpan):
        self.depSta = self.staList[0]
        self.arrSta = self.staList[-1]
        '''
        create train arcs
        :param v_staList:
        :param secTimes:
        :param model:
        :return:
        '''
        minArr = self.dep_LB  # for curSta
        maxArr = self.dep_UB  # for curSta
        '''
        create arcs involving node s
        '''
        self.arcs['s_', self.staList[0] + '_'] = {}
        for t in range(minArr, maxArr):
            self.arcs['s_', self.staList[0] + '_'][t] = {}  # dep-arr在node t的弧集，固定区间运行时分默认只有一个元素
            self.arcs['s_', self.staList[0] + '_'][t][0] = model.addVar(vtype=GRB.BINARY,
                                                                   name=self.traNo + '_' + 's' + '_'
                                                                        + self.staList[0] + '_time_' + str(t))
        '''
        create arcs between real stations
        '''
        for i in range(len(self.staList) - 1):
            curSta = self.staList[i]
            nextSta = self.staList[i + 1]
            # virtual dual stations
            curSta_dep = curSta + "_"
            nextSta_arr = "_" + nextSta
            nextSta_dep = nextSta + "_"

            secRunTime = secTimes[curSta, nextSta]  # 区间运行时分

            # 创建两个弧, 一个运行弧，一个停站弧
            '''
            curSta_dep-->nextSta_arr区间运行弧
            '''
            self.arcs[curSta_dep, nextSta_arr] = {}
            secRunTime += self.stop_addTime  # 添加停车附加时分

            if self.linePlan[curSta] == 1:  # 本站停车， 加起停附加时分
                secRunTime += self.start_addTime
            # 设置d-a的区间运行弧
            for t in range(minArr, maxArr):
                if t + secRunTime > TimeSpan:  # 如果当前t已超过horizon，直接break
                    break
                self.arcs[curSta_dep, nextSta_arr][t] = {}  # dep-arr在node t的弧集，固定区间运行时分默认只有一个元素
                self.arcs[curSta_dep, nextSta_arr][t][secRunTime] = model.addVar(vtype=GRB.BINARY,
                                                                                 name=self.traNo + '_' + curSta_dep + '_'
                                                                                      + nextSta_arr + '_time_' + str(
                                                                                     t))
                # 灵活运行时分弧，散点为1
                # self.arcs[curSta_dep, nextSta_arr][t][secRunTime + 1] = model.addVar(vtype=GRB.CONTINUOUS,
                #                                                                  name=self.traNo + '_' + curSta_dep + '_'
                #                                                                       + nextSta_arr + '_time_' + str(
                #                                                                      t))
            # update cur time window
            minArr += secRunTime
            maxArr = min(maxArr + secRunTime, TimeSpan)
            if minArr > maxArr:  # 如果最早的弧当前都要越界, break掉
                break
            '''
            nextSta_arr-->nextSta_dep车站停站弧
            '''
            if i + 1 == len(self.staList) - 1:  # 若停站, 但已经是最后一个站了，不需停站弧
                break

            self.arcs[nextSta_arr, nextSta_dep] = {}
            if self.linePlan[nextSta] == 1: # 该站停车，创建多个停站时间长度的停站弧
                for t in range(minArr, maxArr):
                    if t + self.min_dwellTime > TimeSpan:  # 当前t加上最短停站时分都超了，break掉
                        break
                    self.arcs[nextSta_arr, nextSta_dep][t] = {}
                    for span in range(self.min_dwellTime, self.max_dwellTime):
                        if t + span > TimeSpan:
                            break
                        self.arcs[nextSta_arr, nextSta_dep][t][span] = model.addVar(vtype=GRB.BINARY,
                                                                                    name=self.traNo + '_' + nextSta_arr
                                                                                         + nextSta_dep + '_time_' + str(
                                                                                        t))
            else: # 该站不停车，只创建一个竖直弧，长度为0
                for t in range(minArr, maxArr):
                    self.arcs[nextSta_arr, nextSta_dep][t] = {}
                    self.arcs[nextSta_arr, nextSta_dep][t][0] = model.addVar(vtype=GRB.BINARY,
                                                                             name=self.traNo + '_' + nextSta_arr
                                                                                  + nextSta_dep + '_time_' + str(
                                                                                 t))
            # update cur time window
            minArr += self.min_dwellTime
            maxArr = min(maxArr + self.max_dwellTime, TimeSpan)
            if minArr > maxArr:
                break
        '''
        create arcs involving node t
        '''
        self.arcs['_' + self.staList[-1], '_t'] = {}
        for t in range(minArr, maxArr):
            self.arcs['_' + self.staList[-1], '_t'][t] = {}  # dep-arr在node t的弧集，固定区间运行时分默认只有一个元素
            self.arcs['_' + self.staList[-1], '_t'][t][0] = model.addVar(0, 1, vtype=GRB.BINARY,
                                                                    name=self.traNo + '_' + self.staList[-1] + '_'
                                                                         + 't' + '_time_' + str(t))


