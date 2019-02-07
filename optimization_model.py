#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  2 19:52:13 2019

@author: Kartik
"""

import networkx as nx
import os
import gurobipy as grb

#PREPROCESSING
#Read in the .txt file
os.getcwd()
os.chdir('/Users/apple/Google Drive/A&M/Spring 2019/Max clique')

def preprocess(file):
    '''takes in the name of the file as a string and returns a list of edges'''
    f = open(file, 'r')
    lines = f.read().split("\n")
    col = [line.split() for line in lines] #split each line into a list
    condition = 'e' #all edges start with e
    wanted_list_3 = [i for i in col if(len(i) == 3)] #by len as some line may be empty
    wanted_list_e = [j for j in wanted_list_3 if(j[0] == condition)] #filter based on e
    wanted_list_s = [l[1:] for l in wanted_list_e] #only keep the edges
    wanted_list = [list(map(int, i)) for i in wanted_list_s] #convert string to int
    return (wanted_list)


def create_graph(edge_list):
    '''Takes in the list of edges as input and returns a graph'''
    elist = [tuple(x) for x in edge_list] #convert sub elements to tuple as req by networkx
    G = nx.Graph()
    G.add_edges_from(elist)
    print(G.number_of_nodes())
    return (G)


#GREEDY MAXIMAL INDEPENDENT SETS
#Get min degree vertex subroutine
def get_min_degree_vertex(Residual_graph):
    '''Takes in the residual graph R and returns the node with the lowest degree'''
    degrees = [val for (node, val) in Residual_graph.degree()]
    node = [node for (node, val) in Residual_graph.degree()]
    node_degree = dict(zip(node, degrees))
    return (min(node_degree, key = node_degree.get))

#Main greedy algorithm
def greedy_init(G):
    '''Takes in the graph and returns maximal independent sets'''
    n = G.number_of_nodes()                 #Storing total number of nodes in 'n'
    max_ind_sets = []                       #initializing a list that will store maximum independent sets
    for j in range(1, n+1):
        R = G.copy()                        #Storing a copy of the graph as a residual
        neigh = [n for n in R.neighbors(j)] #Catch all the neighbours of j
        R.remove_node(j)                    #removing the node we start from
        max_ind_sets.append([j])
        R.remove_nodes_from(neigh)          #Removing the neighbours of j
        if R.number_of_nodes() != 0:
            x = get_min_degree_vertex(R)
        while R.number_of_nodes() != 0:
            neigh2 = [m for m in R.neighbors(x)]
            R.remove_node(x)
            max_ind_sets[j-1].append(x)
            R.remove_nodes_from(neigh2)
            if R.number_of_nodes() != 0:
                x = get_min_degree_vertex(R)
    return(max_ind_sets)


#CHECK1: IF IT IS AN INDEPENDENT SET
def check1(max_ind_sets, check=True):
    n = G.number_of_nodes()
    check  = True
    m_sets = max_ind_sets
    all_v = list(range(1,n+1))
    for i in m_sets:
        i.sort()
        s_all = all_v.copy()
        s1_all = s_all.copy()
        for j in i:
            for k in s_all:
                if G.has_edge(j,k):
                    s1_all.remove(k)
            s_all = s1_all.copy()
        if s_all != i:
            check = False
            print(check, s_all, i)
            break
        else:
            print('Test succesfull')

#Check2: if max independent sets has all the vertices
def check2(max_ind_sets, check = True):
    n = G.number_of_nodes()
    check2 = []
    for a in max_ind_sets:
        for b in a:
            check2.append(b)
    verified = set(check2)
    if len(verified) != n:
        check = False
        print(check)
    else:
        print('Test succesfull')

edge_list = preprocess('san200_0.9_1.txt')
G = create_graph(edge_list)
nx.draw(G)
max_ind_sets = greedy_init(G)
check1(max_ind_sets)
check2(max_ind_sets)
n = G.number_of_nodes()

#MODELLING
#GUROBI
#RMP MODEL
y_var = {}
temp = {}
#Create a set I' for obj function summation
set_I = range(1, n+1)

#Create a set for constraint summation
set_II = max_ind_sets
#Define an optimization model
rmp_model = grb.Model(name="RMP")
rmp_model.setParam(grb.GRB.Param.Presolve, 0)
rmp_model.setParam('OutputFlag', False)
#Create a continous decision variable 'y'
for i in set_I:
    y_var[i] = rmp_model.addVar(obj=1, lb=0.0, vtype=grb.GRB.CONTINUOUS, name="y_var[%d]"%i)
#Create constraints
# >= constraints
x = 0
for i in set_II:
    x = x+1
    var = [y_var[k] for k in i]
    coef = [1] * len(i)
    temp[x] = rmp_model.addConstr(grb.LinExpr(coef,var), ">", 1, name="temp[%d]"%x)
#Objective Function
objective = grb.quicksum(y_var[j]
                         for j in set_I)
rmp_model.setObjective(objective, grb.GRB.MINIMIZE)
rmp_model.write('rmp_day2.lp')

#get dual values as a vector which is the d vector
def get_dual(rmp_model):
    l_constraints = rmp_model.getConstrs()
    dual = [c.Pi for c in l_constraints]
    return(dual)

#CGSP

#set of vertices
set_III = edge_list
#Define an optimization model
cgsp_model = grb.Model(name = "CGSP")
cgsp_model.setParam(grb.GRB.Param.Presolve, 0)
cgsp_model.setParam('OutputFlag', False)
#Create a biniary decision variable
x_var = {}
for j in set_I:
    x_var[j] = cgsp_model.addVar(obj = 1, vtype = grb.GRB.BINARY, name = "x_var[%d]"%j)
#create constraints
temp2 = {}
y = 0
for (i,j) in set_III:
    y = y+1
    var1 = [x_var[i]]
    coef1 = [1]
    var2 = [x_var[j]]
    coef2 = [1]
    expr = grb.LinExpr(coef1, var1)
    expr.addTerms(coef2, var2)
    temp2[y] = cgsp_model.addConstr(expr, grb.GRB.LESS_EQUAL, 1, "temp2[%d]"%y)
cgsp_model.write('day2_cgsp.lp')


def update_obj(dual):
    var3 = [x_var[j] for j in set_I]
    coef3 = [dual[j-1] for j in set_I]
    objective2 = grb.LinExpr(coef3, var3)
    cgsp_model.setObjective(objective2, grb.GRB.MAXIMIZE)
    cgsp_model.update()
    ob = cgsp_model.getObjective()
    #print(ob)
    cgsp_model.write('cgsp.lp')


#Column generation
K = len(set_I) + 1

while True:
    rmp_model.optimize()
    print('RMP_Objective : ', rmp_model.ObjVal)
    rmp_model.write('day2_rmp.mps')
    dual = get_dual(rmp_model)  #get dual from the 'rmp_model'
    update_obj(dual)
    cgsp_model.optimize()
    x_values = cgsp_model.x
    print('CGSP_Objective : ', cgsp_model.ObjVal)
    if cgsp_model.ObjVal <=1.001:
        break
    else:
        col = grb.Column()
        for i in range(1,n):
            col.addTerms(x_values[i-1], temp[i])
        y_var[K] = rmp_model.addVar(obj=1, vtype=grb.GRB.CONTINUOUS, name="y_var[%d]"%K, column = col)
        rmp_model.update()
        rmp_model.write('updated.lp')
        K += 1
