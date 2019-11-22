#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 17 10:10:46 2019

@author: frankchao
AI introduction ntpu 2019
"""

"""
Please use the following search algorithms to solve the 8-queen problem.

(a) Iterative-Deepening Search (IDS)
(b) Uniform-Cost Search
(c) Greedy Best-First Search
(d) A* search
(e) Recursive Best-First Search (RBFS)

Input： A state of random layout of one queen on the board.
Goal state：8 queens are on the board, none attacked

Output for each algorithm：
(a) The number of movements (state changes) from the initial state to the goal state.
(b) The maximum number of states ever saved in the memory during the process.

"""

import numpy as np

frontier=[]                 
expanded_node=[]           # 正在展開的state
state_changes=0            # 記錄展開過多少個state
MaxnumState=0              # maximun number of states ever saved in memory
find=False                 # 如果找到goal,find = True
Rbfsmax=0                  # 記錄rbfs最多的記憶的node
goal=None          


def countArray(matrix):              #計算有多少chess
    count=0
    for ii in range(8):
        for jj in range(8):
            if matrix[ii][jj]=='〒':
                count+=1
    return count            

def children_left(matrix):
    find=False
    for i in range(8):                #找到最左邊有chess的column i
        for j in range(8):
            if matrix[j][i]=='〒':
                find=True
                break
        if find==True:
            break
    children=[]
    for k in range(8):
        matrix2=matrix.copy()
        if k!=j and k!=j-1 and k!=j+1:  #在column i的左邊column加入chess, 但是左上、左邊、左下不加 , 根據規則  
           if i-1>=0: 
              matrix2[k][i-1]='〒'
              children.append(matrix2)
    return children       

def children_right(matrix):
    find=False
    for i in range(7,-1,-1):            #找到最右邊有chess的column i
        for j in range(8):
            if matrix[j][i]=='〒':
                find=True
                break
        if find==True:
            break
    children=[]
    for k in range(8):
        matrix2=matrix.copy()
        if k!=j and k!=j-1 and k!=j+1:  #在column i的右邊column加入chess, 但是右上、右邊、右下不加 , 根據規則  
           if i+1<8: 
              matrix2[k][i+1]='〒'
              children.append(matrix2)
    return children       

class Node:

    def __init__(self,state,level):
        self.state = state        #一個state
        self.expanded = False     #被展開過 = True
        self.children = []        #加入展開後的子state
        self.parent=[]            #父節點(state)
        self.level=level          #紀錄它是第幾層
        self.cost=0
        self.keep=None            #(rbfs)
        self.value=None           #(rbfs)
    
    def add_children(self):       #加入children
        '''
        不add_children的情況
        (i,j)-> (i-1,j-1),(i,j-1),(i+1,j-1)   
             -> (i+1,j+1),(i,j+1),(i-1,j+1)
             
        先加左邊，再加右邊
        如果第0個column已經有棋子了,再從initial＿state的右邊add＿children
        '''
        LorR=0
        for i in range(8):
            if self.state[i][0]=='〒':          
                 LorR=1                        
        if  countArray(self.state) ==8:         #如果已經有8個chess,則沒有children
            return None
        if  LorR==0:                            #如果第0個column沒有棋子
            add=children_left(self.state)       #就加左邊
            for n in add:
               left=Node(n,self.level+1)
               left.cost=cost(left,expanded_node)
               left.parent.append(self.state)        #記錄parent
               self.children.append(left)            #記錄children
        else:    
            add=children_right(self.state)           #如果第0個column沒有棋子
            for n in add:                            #就加右邊
               right=Node(n,self.level+1)
               right.cost=cost(right,expanded_node)
               right.parent.append(self.state)       #記錄parent
               self.children.append(right)           #記錄children
    def __repr__(self):          
        return str(self.state)    #print會印出state


def heuristic(matrix):
    '''
    計算所有棋子會互相攻擊的數目
    為了方便計算，同一組會重複算
    並減掉盤面上旗子的數目(越多棋子越好)
    滿足admissible:h(n)<=h* , h*為count加總
    '''
    count=0
    chess=[]
    for i in range(8):
        for j in range(8):
            if matrix[i][j]=='〒':
                chess.append([i,j])              #找到所有的chess
    for k in chess:
        for i in range(1,7):
            if k[0]-i>=0 and k[1]-i>=0:          #計算左上
                if [k[0]-i,k[1]-i] in chess:
                    count+=1
            if k[0]+i<=7 and k[1]+i<=7:          #計算左下 
                if [k[0]+i,k[1]+i] in chess:
                    count+=1
            if k[0]-i>=0 and k[1]+i<=7:          #計算右上
                if [k[0]-i,k[1]+i] in chess:
                    count+=1
            if k[0]+i<=7 and k[1]-i>=0:          #計算右下
                if [k[0]+i,k[1]-i] in chess: 
                    count+=1        
        for j in range(1,8):
            if k[0]-j>=0:                        #計算上方
                if [k[0]-j,k[1]] in chess:
                    count+=1
            if k[0]+j<=7:                        #計算下方
                if [k[0]+j,k[1]] in chess:
                    count+=1        
            if k[1]-j>=0:                        #計算左方
                if [k[0],k[1]-j] in chess:
                    count+=1
            if k[1]+j<=7:                        #計算右方
                if [k[0],k[1]+j] in chess:
                    count+=1               
    return count-countArray(matrix)             

def cost(Node,expand):
    num=0
    path=[]
    path.append(Node) 
    while Node.parent!=[]:                       #找到path上所有的node,記錄cost
        for ii in range(0,len(expand)):
            if (np.array(expand[ii].state).reshape(8,8)==np.array(Node.parent).reshape(8,8)).all():
                path.append(expand[ii])
                break
        Node=expand[ii]
    for j in path:                              
        num+=heuristic(j.state)+countArray(j.state)  #加總所有cost, cost為 heuristic(j.state)+countArray(j.state)
    return num 
        
 
def test(matrix):                      #測試是否所有棋子皆為non-attack
    chess=[]
    for i in range(8):
        for j in range(8):
            if matrix[i][j]=='〒':
                chess.append([i,j])           
    for i in range(8):                  #檢查row有沒有重複的chess
        count=0
        for j in range(8):
           if  matrix[i][j]=='〒':
               count+=1
           if count>1:
             return False              
    for i in range(8):                 #檢查column有沒有重複的chess
        count=0
        for j in range(8):
           if  matrix[j][i]=='〒':
               count+=1
           if count>1:
             return False  
        
    for k in chess :                   #檢查斜邊
        count=0
        for i in range(1,8):
            if k[0]-i>=0 and k[1]-i>=0:
                if [k[0]-i,k[1]-i] in chess:
                    count+=1
            if k[0]+i<=7 and k[1]+i<=7:
                if [k[0]+i,k[1]+i] in chess:
                    count+=1
        if count>0:
            return False
    for k in chess :                   #檢查斜邊
        count=0
        for i in range(1,8):
            if k[0]-i>=0 and k[1]+i<=7:
                if [k[0]-i,k[1]+i] in chess:
                    count+=1
            if k[0]+i<=7 and k[1]-i>=0:
                if [k[0]+i,k[1]-i] in chess:
                    count+=1 
        if count>0:
            return False       
    for k in chess :                   #檢查斜邊
        count=0
        for i in range(1,8):
            if k[0]-i>=0 and k[1]-i>=0:
                if [k[0]-i,k[1]-i] in chess:
                    count+=1
            if k[0]+i<=7 and k[1]+i<=7:
                if [k[0]+i,k[1]+i] in chess:
                    count+=1
        if count>0:
            return False
    for k in chess :                   #檢查斜邊
        count=0
        for i in range(1,8):
            if k[0]-i>=0 and k[1]+i<=7:
                if [k[0]-i,k[1]+i] in chess:
                    count+=1
            if k[0]+i<=7 and k[1]-i>=0:
                if [k[0]+i,k[1]-i] in chess:
                    count+=1 
        if count>0:
            return False           
    return True    
         

class Iterative_Deepening_Search:
    
    '''
    IDS Algorithm
    LIFO 先加入的後展開-> 加入順序 右左下上(進入frontier的順序)
    node展開的順序為上下左右(*往上的state先展開 再來*往下的state ...)
    goal test when insert
    
    '''
    delete=0
    def __init__(self,initial):
        global find
        self.start=initial
        find=self.stack()             #如果找到goal,find會被改成true
        
    def add_frontier(self):
        global frontier
        global expanded_node
        global state_changes
        start = self.start
        if test(start.state)==True and countArray(start.state)==8:   #如果找到goal
            path=[]
            path.append(start)  
            while start.parent!=[]:                                  #找它的parent,直到initial
                for ii in range(0,len(expanded_node)):
                    if (np.array(expanded_node[ii].state).reshape(8,8)==np.array(start.parent).reshape(8,8)).all():
                       path.append(expanded_node[ii])
                       break
                start=expanded_node[ii]
            path.reverse()    
            for j in range(0,len(path)):
                if isinstance(path[j].state,np.ndarray)!=True:
                    path[j].state=np.array(path[j].state).reshape(8,8)
                print('move:',j)
                print(path[j])
            return True   
        frontier.append(start)               #沒找到goal,則加入forntier
        
    def stack(self):
        '''
        stack 後進先出
        因此把forntier由後展開
        並把展開的state從frontier刪除
        並把展開state的children加入frontier
        '''
        global frontier
        global expanded_node
        global MaxnumState
        global state_changes
        expanded_node.append(frontier[-1])
        state_changes+=1
        del frontier[-1]
        expanded_node[-1].expanded = True
        expanded_node[-1].add_children()
        for n in expanded_node[-1].children:           #把展開state的children加入frontier
            if len(expanded_node[-1].children)==0:break
            if not n.expanded:
               self.start=n
               f=self.add_frontier()
               if len(expanded_node)+len(frontier)>MaxnumState:  #記錄maxnumstate           
                   MaxnumState=len(frontier)+len(expanded_node)  
               if f==True:                                       #如果在add_frontier過程中找到goal,f會＝true
                 return f   
       
class Uniform_Cost_Search:
    movement=[]                         #記錄最多有幾個node
    def __init__(self,initial):
       global find
       self.start=initial
       find=self.priority_queue()
       
    def add_frontier(self):
        global frontier
        global expanded_node
        start = self.start
        frontier.append(start)
        Uniform_Cost_Search.movement.append(start)
        
    def pop_off(self):                                                           #在pop_off的時候才做goal test
        global frontier
        global expanded_node
        global state_changes
        if test(frontier[0].state)==True and countArray(frontier[0].state)==8:   #如果通過test且chess數目為8個，則為goal
            path=[]
            path.append(frontier[0].state)  
            start=frontier[0]
            while start.parent!=[]:                                              #找它的parent,直到initial
                for ii in range(0,len(expanded_node)):
                    if (np.array(expanded_node[ii].state).reshape(8,8)==np.array(start.parent).reshape(8,8)).all():
                       path.append(expanded_node[ii])
                       break
                start=expanded_node[ii]
            path.reverse()    
            for j in range(0,len(path)):
                if isinstance(path[j],np.ndarray)!=True:
                    path[j].state=np.array(path[j].state).reshape(8,8)
                print('move:',j)
                print(path[j])
            return True   
        else:                                                 #否則
            def fsorted(node):                                #把frontier的state按照cost排序
                return node.cost   
            frontier.sort(key=fsorted)  
            expanded_node.append(frontier[0])                 #展開cost最小者 
            state_changes+=1
            del frontier[0]
        
    def priority_queue(self):
        '''
        Uniform-cost Algorithm
        f(Node)=cost(Node)=state.level + heuristic =  加入第幾個棋子＋棋子間攻擊的數目     
        goal test when pop off
        
        '''
        global frontier
        global expanded_node
        global MaxnumState
        f=self.pop_off()          #測試要expand的node是不是goal
        if f==True:
            return f    
        expanded_node[-1].expanded = True     #不是的話就展開
        expanded_node[-1].add_children()
        for n in expanded_node[-1].children:
            if not n.expanded:
               self.start=n
               self.add_frontier()

class Geedy_Bestfirst_Search:
    movement=[]                              #記錄最多有幾個node
    def __init__(self,initial):
        global find
        self.start=initial
        find=self.greedy()
         
    def add_frontier(self):                  #在加入fontier時做goal-test
        global frontier
        global expanded_node
        start = self.start
        if test(start.state)==True and countArray(start.state)==8:   #如果找到goal
            path=[]
            path.append(start) 
            Geedy_Bestfirst_Search.movement.append(start)
            while start.parent!=[]:                                              #找它的parent,直到initial
                for ii in range(0,len(expanded_node)):
                    if (np.array(expanded_node[ii].state).reshape(8,8)==np.array(start.parent).reshape(8,8)).all():
                       path.append(expanded_node[ii])
                       break
                start=expanded_node[ii]
            path.reverse()    
            for j in range(0,len(path)):
                if isinstance(path[j].state,np.ndarray)!=True:
                    path[j].state=np.array(path[j].state).reshape(8,8)
                print('move:',j)
                print(path[j])
            return True   
        frontier.append(start)
        Geedy_Bestfirst_Search.movement.append(start)
        
    def greedy(self):
        
        '''
        Greedy best-first search Algorithm
        h(n)使用heuristic中的定義，也就是Manhattan distance
        當h(n)相同時，先加入frontier的先展開
        h(n)為0時，為solution
        '''
        global frontier
        global expanded_node
        global MaxnumState
        global state_changes
        gmin=100000
        for k in range(0,len(frontier)):           #在frontier中,找h(n)最小者展開
            g=heuristic(frontier[k].state)
            if g<gmin:
                add=k
                gmin=g
        expanded_node.append(frontier[add])
        state_changes+=1
        del frontier[add]
        expanded_node[-1].expanded = True
        expanded_node[-1].add_children()
        for n in expanded_node[-1].children: 
            if not n.expanded:
               self.start=n
               f=self.add_frontier()
               if f==True:                       #如果在add_frontier過程中找到goal,f會＝true
                 return f  

class Astar:
    movement=[]                              #記錄最多有幾個node
    def __init__(self,initial):
        global find
        self.start=initial
        find=self.astar()
         
    def add_frontier(self):                  #在加入fontier時做goal-test
        global frontier
        global expanded_node
        start = self.start
        if test(start.state)==True and countArray(start.state)==8:   #如果找到goal
            path=[]
            path.append(start) 
            Astar.movement.append(start)
            while start.parent!=[]:                                  #找它的parent,直到initial
                for ii in range(0,len(expanded_node)):
                    if (np.array(expanded_node[ii].state).reshape(8,8)==np.array(start.parent).reshape(8,8)).all():
                       path.append(expanded_node[ii])
                       break
                start=expanded_node[ii]
            path.reverse()    
            for j in range(0,len(path)):
                if isinstance(path[j].state,np.ndarray)!=True:
                    path[j].state=np.array(path[j].state).reshape(8,8)
                print('move:',j)
                print(path[j])
            return True   
        frontier.append(start)
        Astar.movement.append(start)
        
    def astar(self):
        
        '''
        Astar_Search Algorithm
        f(n)=g(n)+h(n)
        g(n)的step cost均為1
        h(n)使用heuristic中的定義，也就是Manhattan distance
        當f(n)相同時，先加入frontier的先展開
        f(n)為0時，為solution
        '''
        global frontier
        global expanded_node
        global MaxnumState
        global state_changes
        fmin=100000
        for k in range(0,len(frontier)):           #在frontier中,找f(n)最小者展開
            g=heuristic(frontier[k].state)         #g為heuristic
            f=g+frontier[k].cost                   #frontier[k].cost為 g(n)
            if f<fmin:
                add=k
                fmin=f
        expanded_node.append(frontier[add])
        state_changes+=1
        del frontier[add]
        expanded_node[-1].expanded = True
        expanded_node[-1].add_children()
        for n in expanded_node[-1].children: 
            if not n.expanded:
               self.start=n
               f=self.add_frontier()
               if f==True:                         #如果在add_frontier過程中找到goal,f會＝true
                 return f  

class Rbfs:
    movement=[]                              #記錄有幾個node
    def __init__(self,initial):
        global find
        self.start=initial
        find=self.rbfs()
         
    def add_frontier(self):                  #在加入fontier時做goal-test
        global frontier
        global expanded_node
        global Rbfsmax
        start = self.start
        frontier.append(start)
        Rbfs.movement.append(start)
        if len(Rbfs.movement)>Rbfsmax:       #frontier多1,Rbfs.movement就多1
           Rbfsmax=len(Rbfs.movement)        #Rbfsmax記錄最多的node數目
        goal=Rbfs.goal_test()                #goal test 
        if goal==True:
            return True

    
    @classmethod    
    def goal_test(cls):
        global frontier
        global expanded_node
        global MaxnumState
        global Rbfsmax
        for ii in range(0,len(frontier)):
            if test(frontier[ii].state)==True and countArray(frontier[ii].state)==8:   #如果找到goal
               path=[]
               path.append(frontier[ii]) 
               Rbfs.movement.append(frontier[ii])
               start=frontier[ii]
               while start.parent!=[]:                                                 #找它的parent,直到initial
                 for ii in range(0,len(expanded_node)):
                     if (np.array(expanded_node[ii].state).reshape(8,8)==np.array(start.parent).reshape(8,8)).all():
                       path.append(expanded_node[ii])
                       break
                 start=expanded_node[ii]
               path.reverse()    
               for j in range(0,len(path)):
                 if isinstance(path[j].state,np.ndarray)!=True:
                      path[j].state=np.array(path[j].state).reshape(8,8)
                 print('move:',j)
                 print(path[j])
               return True   
           
    def rbfs(self):
        
        '''
        Recursive Best-First Search (RBFS)
        f(n)=g(n)+h(n)
        g(n)的step cost均為1
        h(n)使用heuristic中的定義，也就是Manhattan distance
        記住次好的goal,轉換時忘掉
        '''
        global frontier
        global expanded_node
        global MaxnumState
        global Rbfsmax
        global state_changes
        if len(frontier)==1 and frontier[-1].level==0 :        #第一次的時候直接展開
            expanded_node.append(frontier[-1])
            state_changes+=1
            del frontier[-1]
            expanded_node[-1].expanded = True
            expanded_node[-1].add_children()
            for n in expanded_node[-1].children: 
              if not n.expanded:
                self.start=n
                f=self.add_frontier()
                if f==True:
                  return True      
            return None               
        if expanded_node[-1].children==[]:
            num=len(expanded_node[-1].children)
            for j in range(0,num):                            #把expanded_node[-1]的children自frontier去除
                del frontier[-1]
                del Rbfs.movement[-1]
            expanded_node[-1].value=10000    
            frontier.append(expanded_node[-1])                #把expanded_node[-1]加回frontier
            del expanded_node[-1]                             #自expanded_node刪除
            return None
        for m in expanded_node[-1].children:                  #如果沒找到goal
           if m.value==None:
              g=heuristic(m.state)                            #記錄children的f(n)
              f=g+cost(m,expanded_node)   
              m.value=f 
        def fsort(x):                                         #按照children的f(n)值排序
             return x.value      
        expanded_node[-1].children.sort(key=fsort)  
        fmin=expanded_node[-1].children[0].value  
        fsecond=expanded_node[-1].children[1].value          
        add=expanded_node[-1].children[0]                     #add為f(n)最小者,並找frontier第二小的值
        if add.value>expanded_node[-1].keep:                  #如果fmin>keep
            expanded_node[-1].value=add.value                 #expanded_node[-1].value=fmin       
            expanded_node[-1].keep=None
            expanded_node[-1].expanded=False 
            num=len(expanded_node[-1].children)
            for j in range(0,num):                            #把expanded_node[-1]的children自frontier去除
                del frontier[-1]
                del Rbfs.movement[-1]
            frontier.append(expanded_node[-1])                #把expanded_node[-1]加回frontier
            del expanded_node[-1]                             #自expanded_node刪除
        else:                         
            add.keep=fsecond                                  #如果fmin<=keep
            expanded_node.append(add)                         #展開add
            state_changes+=1
            if expanded_node[-1].children==[]:
               expanded_node[-1].add_children()
            expanded_node[-1].expanded=True
            for n in expanded_node[-1].children: 
               if not n.expanded:
                 self.start=n
                 f=self.add_frontier()
                 if f==True:
                  return True 
             
def IDS(initial_state,initial):
    print('')
    print('This is IDS')
    print('####################')
    global find
    global MaxnumState
    global state_changes
    for i in range(1,9):                          #在7層時會解開
        if find==True:
            MaxnumState=MaxnumState
            print('The number of state changes:',state_changes)
            state_changes=0
            print('Max states ever saved:',MaxnumState)
            print('############################')
            print('\n')      
            find=False
            MaxnumState=0
            break
        if i==8:                                  
           print('unsolvable')
           print('############################')
           expanded_node.clear()
           frontier.clear()  
           MaxnumState=0
           state_changes=0
           break
        else:   
           frontier.append(Node(initial_state,0))
        while find!=True:                 
           if initial.level==i:                   #如果要展開的state == limit,則不展
              del frontier[-1]
           else:   
              Iterative_Deepening_Search(initial)  
           if len(frontier)!=0:
              initial=frontier[-1]                #展開forntier[-1],後進先出
              for n in expanded_node:             #如果在expanded_node裡有比forntier[-1]層數高的 
                  if n.level>=initial.level: 
                     expanded_node.remove(n)      #就刪除，代表回溯 
           else:
               expanded_node.clear()              #做下一層時，清空expanded_node
               frontier.clear()                   #          清空frontier
               break   
        expanded_node.clear()
        frontier.clear()   

def UCS(initial_state,initial):
    print('This is UCS')
    print('####################')
    global find
    global MaxnumState
    global state_changes
    frontier.append(Node(initial_state,0))
    while len(expanded_node)<10000 :
        if find==True:
           MaxnumState=len(Uniform_Cost_Search.movement)+1       #加1,因initial_state沒算到 
           print('The number of state changes:',state_changes)
           state_changes=0
           print('Max states ever saved:',MaxnumState)
           print('############################')
           print('\n')      
           find=False
           MaxnumState=0
           Uniform_Cost_Search.movement=[]
           expanded_node.clear()
           frontier.clear()
           return None
        else:
           Uniform_Cost_Search(initial)
    print('unsolvable') 
    print('############################')
    find=False
    state_changes=0
    MaxnumState=0
    Uniform_Cost_Search.movement=[]
    expanded_node.clear()
    frontier.clear()

def GREEDY(initial_state,initial):
    print('This is GREEDY')
    print('####################')
    global find
    global MaxnumState
    global state_changes
    frontier.append(Node(initial_state,0))
    while len(expanded_node)==0 or len(expanded_node)<5000 :
        if find==True:
           MaxnumState=len(Geedy_Bestfirst_Search.movement)+1     #加1,因initial_state沒算到 
           print('The number of state changes:',state_changes)
           state_changes=0
           print('Max states ever saved:',MaxnumState)
           print('############################')
           print('\n')      
           find=False
           MaxnumState=0
           expanded_node.clear()
           frontier.clear()
           return None
        else:
           Geedy_Bestfirst_Search(initial) 
    print('unsolvable') 
    find=False
    MaxnumState=0
    state_changes=0
    Geedy_Bestfirst_Search.movement=[]
    expanded_node.clear()
    frontier.clear()

def ASTAR(initial_state,initial):
    print('This is ASTAR')
    print('####################')
    global find
    global MaxnumState
    global state_changes
    frontier.append(Node(initial_state,0))
    while len(expanded_node)==0 or len(expanded_node)<100 :
        if find==True:
           MaxnumState=len(Astar.movement)+1                      #加1,因initial_state沒算到 
           print('The number of state changes:',state_changes)
           state_changes=0
           print('Max states ever saved:',MaxnumState)
           print('############################')
           print('\n')      
           find=False
           MaxnumState=0
           expanded_node.clear()
           frontier.clear()
           return None
        else:
           Astar(initial)         
    print('unsolvable') 
    find=False
    MaxnumState=0
    state_changes=0
    Astar.movement=[]
    expanded_node.clear()
    frontier.clear()

def RBFS(initial_state,initial):
    print('This is RBFS')
    print('####################')
    global find
    global MaxnumState
    global Rbfsmax 
    global state_changes
    initial.keep=100000                                          #把initial的keep設為一個大的數字
    frontier.append(initial) 
    while len(expanded_node)<30 :
        if find==True:
           MaxnumState=Rbfsmax+1                                 #加1,因initial_state沒算到 
           print('The number of state changes:',state_changes)
           state_changes=0
           print('Max states ever saved:',MaxnumState)
           print('############################')
           print('\n')      
           find=False
           MaxnumState=0
           Rbfs.movement=[]
           Rbfsmax=0
           expanded_node.clear()
           frontier.clear()
           return None
        else:
           Rbfs(initial)         
    print('unsolvable') 
    find=False
    MaxnumState=0
    state_changes=0
    Rbfs.movement=[]
    Rbfsmax=0
    expanded_node.clear()
    frontier.clear()    

def main():
    '''
    (0,0)(0,1)(0,2)........(0,7)
    (1,0)..................(1,7)
    (2,0)..................(2,7)
      .
      .
      .
      .
    (7,0)..................(7,7)  
    
    '〒'代表chess
    
    '''    
    x,y=map(int, input('請輸入x.y座標:').split())
    origin=np.array([['0','0','0','0','0','0','0','0'],['0','0','0','0','0','0','0','0']
                     ,['0','0','0','0','0','0','0','0'],['0','0','0','0','0','0','0','0'],
                     ['0','0','0','0','0','0','0','0'],['0','0','0','0','0','0','0','0']
                     ,['0','0','0','0','0','0','0','0'],['0','0','0','0','0','0','0','0']],dtype=str)
    origin[x][y]='〒'
    initial_state=origin
    initial=Node(initial_state,0)
    frontier.append(initial)
    IDS(initial_state,initial)              #IDS      結果
    UCS(initial_state,initial)              #UCS      結果
    GREEDY(initial_state,initial)           #GREEDY   結果
    ASTAR(initial_state,initial)            #ASTAR    結果
    RBFS(initial_state,initial)             #RBFS     結果
   
if __name__=='__main__':
    main()      





