#!/usr/bin/env python3
import numpy as np
import osqp

from std_msgs.msg import String
import matplotlib 
# matplotlib.use('Qt5Agg')
from math  import *
from scipy import sparse
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas as pd
import sys
np.set_printoptions(threshold=sys.maxsize)
from minimumsnap_route.srv import *

import rospy

plt.cla()
class minimum_snap():
    def __init__(self):
        self.max_fly_speed = 2 #single dimension
        self.max_fly_acc = 2
        self.n_order = 7 ; #多项式次数
        self.rank = 4; #求导优化次数
        self.segment_time=0.0001
        self.Q_weight=1;
        self.fly_speed=1;

    def calc_dist(self,point1,point2):
        return  sqrt(pow(point1[0]-point2[0],2)
                    +pow(point1[1]-point2[1],2)
                    +pow(point1[2]-point2[2],2))

    def get_total_length_waypoints(self,waypoints_ptc,waypoints_rows):
        total_length=0;
        for i in range (0,waypoints_rows-1):
            total_length+=self.calc_dist(waypoints_ptc[i],waypoints_ptc[i+1])
        return total_length

    def get_time_seq(self,waypoints_ptc,fly_speed,total_length):
        waypoints_rows=waypoints_ptc.shape[0]
        tmp = np.zeros([waypoints_rows,1])
        total_fly_time = total_length/fly_speed;
        time_stamp = 0;
        for i in range(0,waypoints_rows-1):
            length=self.calc_dist(waypoints_ptc[i],waypoints_ptc[i+1]) 
            time_ratio  = length/total_length;
            time_stamp += time_ratio*total_fly_time;
            #print("{} length : {} time_length {} time_stamp {} total length {}".format(i,length,time_ratio*total_fly_time,time_stamp,total_length));
            tmp[i+1]=time_stamp; 
        tmp=tmp/total_fly_time
        return tmp

    class contriant_t(object):
        A=np.array([])
        l=np.array([])
        u=np.array([])
        
    def blkdiag(self,src,new):
        if src.shape[0]!=0:
            tmp = np.zeros([src.shape[0]+new.shape[0],src.shape[1]+new.shape[1]])
            tmp[0:src.shape[0],0:src.shape[1]]=src
            tmp[src.shape[0]:,src.shape[1]:] = new
            return tmp
        else:
            tmp = np.zeros([new.shape[0],new.shape[1]])
            tmp = new
            #print(new)
            return tmp
    def prod(self,num1,num2):
        ans =1 
        if(num1<=num2):
            for i in range(num1,num2+1):
                ans*=i
        else:
            ans = 1
        return ans      
    def get_Q_matrix(self,n_order,rank,t1,t2,Q_weight):   
        Q=np.zeros([n_order+1,n_order+1])
        for r in range(rank+1,n_order+1+1):
            for c in range(rank+1,n_order+1+1):
                tk = pow(t2,r+c-2*rank+1)-pow(t1,r+c-2*rank+1)
                Q[r-1,c-1]=self.prod((r-rank+1),r)*self.prod((c-rank+1),c)/(r+c-2*rank+1)*2*tk    
        Q*=Q_weight    
        return Q 
    def calc_coeffieient(self,t,n_order,r):
        tvec= np.zeros([1,n_order+1])
        for i in range(r+1,n_order+1+1):
            tvec[0,i-1]=self.prod(i-r,i-1)*pow(t,i-r-1)
        return tvec

    def get_coefficient_matrixes(self,waypoints_prt,n_poly,n_order,rank,Time,Q_weigtht,start_pva,end_pva,Q_weight):

        num_of_coefficients=n_order+1
        pos_constraint=self.contriant_t()
        vcc_continuity_constraint=self.contriant_t()
        vcc_constraint=self.contriant_t()
        pos_continuity_constraint=self.contriant_t()
        acc_continuity_constraint=self.contriant_t()
        acc_constraint=self.contriant_t()        
        
        range_P=0.4
        big_Q = np.array([])

        for i in range(n_poly):
            t1 = Time[i];t2=Time[i+1];
            big_Q = self.blkdiag(big_Q,self.get_Q_matrix(n_order,rank,t1,t2,Q_weight));

        pos_constraint.A=np.resize(pos_constraint.A,[n_poly+1,n_poly*num_of_coefficients]);
        pos_constraint.l=np.resize(pos_constraint.l,[n_poly+1,1])
        pos_constraint.u=np.resize(pos_constraint.u,[n_poly+1,1])

        for i in range(n_poly):
            x=waypoints_prt[i]
            t=Time[i]
            pos_constraint.A[i:i+1,i*num_of_coefficients:(i+1)*num_of_coefficients]=self.calc_coeffieient(t,n_order,0);
            pos_constraint.l[i]=x-range_P
            pos_constraint.u[i]=x+range_P
            if i==0:
                pos_constraint.l[i]=x
                pos_constraint.u[i]=x       
        i = n_poly
        x=waypoints_prt[i]
        t=Time[i]
        pos_constraint.A[i:i+1,(i-1)*num_of_coefficients:(i)*num_of_coefficients]=self.calc_coeffieient(t,n_order,0);
        pos_constraint.l[i]=x
        pos_constraint.u[i]=x
        del i
        
    #pos_continuity_constraint
        pos_continuity_constraint.A=np.resize(pos_continuity_constraint.A,[n_poly*2,n_poly*num_of_coefficients]);
        pos_continuity_constraint.l=np.resize(pos_continuity_constraint.l,[n_poly*2,1])
        pos_continuity_constraint.u=np.resize(pos_continuity_constraint.u,[n_poly*2,1])
        for i in range(n_poly-1):
            tmp = self.calc_coeffieient(Time[i+1],n_order,0)
            constrain_vec = np.zeros([1,num_of_coefficients*2])
            constrain_vec[:,0:num_of_coefficients]=tmp
            constrain_vec[:,num_of_coefficients:num_of_coefficients*2]=-tmp
            pos_continuity_constraint.A[i:i+1,num_of_coefficients*i:num_of_coefficients*i+num_of_coefficients*2]=constrain_vec
        
    #vcc_continuity_constraint    
        vcc_continuity_constraint.A=np.resize(vcc_continuity_constraint.A,[n_poly-1,n_poly*num_of_coefficients])
        vcc_continuity_constraint.l=np.resize(vcc_continuity_constraint.l,[n_poly-1,1])
        vcc_continuity_constraint.u=np.resize(vcc_continuity_constraint.u,[n_poly-1,1]) 
        for i in range(n_poly-1):
            tmp = self.calc_coeffieient(Time[i+1],n_order,1)
            constrain_vec = np.zeros([1,num_of_coefficients*2])
            constrain_vec[:,0:num_of_coefficients]=tmp
            constrain_vec[:,num_of_coefficients:num_of_coefficients*2]=-tmp
            vcc_continuity_constraint.A[i:i+1,num_of_coefficients*i:num_of_coefficients*i+num_of_coefficients*2]=constrain_vec
    #acc_continuity_constraint  
        acc_continuity_constraint.A=np.resize(acc_continuity_constraint.A,[n_poly-1,n_poly*num_of_coefficients])
        acc_continuity_constraint.l=np.resize(acc_continuity_constraint.l,[n_poly-1,1])
        acc_continuity_constraint.u=np.resize(acc_continuity_constraint.u,[n_poly-1,1])       
        for i in range(n_poly-1):
            tmp = self.calc_coeffieient(Time[i+1],n_order,2)
            constrain_vec = np.zeros([1,num_of_coefficients*2])
            constrain_vec[:,0:num_of_coefficients]=tmp
            constrain_vec[:,num_of_coefficients:num_of_coefficients*2]=-tmp
            acc_continuity_constraint.A[i:i+1,num_of_coefficients*i:num_of_coefficients*i+num_of_coefficients*2]=constrain_vec

            
        acc_constraint.A=np.resize(acc_constraint.A,[n_poly+1,n_poly*num_of_coefficients])
        acc_constraint.l=np.resize(acc_constraint.l,[n_poly+1,1])
        acc_constraint.u=np.resize(acc_constraint.u,[n_poly+1,1]) 
        for i in range(n_poly):
            t=Time[i]
            acc_constraint.A[i:i+1,i*num_of_coefficients:(i+1)*num_of_coefficients]=self.calc_coeffieient(t,n_order,2)*-1
            acc_constraint.l[i]=-self.max_fly_acc
            acc_constraint.u[i]=self.max_fly_acc
        i = n_poly
        t=Time[i]
        acc_constraint.A[i:i+1,(i-1)*num_of_coefficients:(i)*num_of_coefficients]=self.calc_coeffieient(t,n_order,2)*-1
        acc_constraint.l[i]=-self.max_fly_acc
        acc_constraint.u[i]=self.max_fly_acc
        acc_constraint.l[0]=start_pva[2]
        acc_constraint.u[0]=start_pva[2]
        acc_constraint.l[n_poly]=end_pva[2]
        acc_constraint.u[n_poly]=end_pva[2]
        del i

        def constraints_concatenate(*sub_constraints):
            ALL=self.contriant_t()
            for constrains in sub_constraints:
                if ALL.A.shape[0]==0:
                    ALL.A=constrains.A
                    ALL.l=constrains.l
                    ALL.u=constrains.u
                ALL.A = np.concatenate((ALL.A,constrains.A),0)
                ALL.l = np.concatenate((ALL.l,constrains.l),0)
                ALL.u = np.concatenate((ALL.u,constrains.u),0)
            return ALL
        ALL=constraints_concatenate(
                                    pos_constraint
                                    ,pos_continuity_constraint
                                    ,vcc_continuity_constraint
                                    # ,vcc_constraint
                                    ,acc_constraint
                                    )
        return big_Q ,ALL

    def calc_polys_points (self,cofficients_of_polys,tt,n_order): 
        val = np.zeros([tt.shape[0]])
        for j in range(tt.shape[0]):
            for i in range(n_order+1):
                val[j]=val[j]+cofficients_of_polys[i]*pow(tt[j],i)
                #print(cofficients_of_polys[i])
        return val

    def calc_polys_points (self,cofficients_of_polys,tt,n_order,n_poly): 
        val = np.zeros([tt.shape[0]])
        wp_id=[]
        for j in range(tt.shape[0]):
            for i in range(n_order+1):
                val[j]=val[j]+cofficients_of_polys[i]*pow(tt[j],i)
                wp_id.append(n_poly)
        return val,wp_id

    class PVA():
        def __init__(self,p,v,a):
            self._p = p
            self._v = v
            self._a = a

    status='solved'


    def waypoints_corride(self,waypoints):
        
        waypoints_copy = waypoints
        waypoints_rows = waypoints.shape[0]
        waypoints_new = np.array([])
        for i in range(waypoints_rows-1):
            if(self.calc_dist(waypoints_copy[i],waypoints_copy[i+1]>10)):
                waypoints_new=np.append(waypoints_new,np.linspace(waypoints_copy[i],waypoints_copy[i+1],2,endpoint=False)) 
            else:
                waypoints_new=np.append(waypoints_new,waypoints_copy[i])
        waypoints_new=np.append(waypoints_new,waypoints_copy[-1])   
        return np.reshape(waypoints_new,(-1,4))
        

    def minimumsnap(self,waypoints,n_order,rank,fly_speed,start_pva,end_pva,Q_weight):
        xyz_final = np.array([])
        T_draw = np.array([])

        self.status='solved'

        waypoints=self.waypoints_corride(waypoints)

        # print(waypoints)

        for col_index in range(3):
        
            waypoints_prt = waypoints[:,col_index]
            
            waypoints_rows = waypoints.shape[0]
            
            time_vec=self.get_time_seq(waypoints,fly_speed,self.get_total_length_waypoints(waypoints,waypoints_rows))
                
            n_poly = waypoints_prt.shape[0]-1;

            startpva=np.array([start_pva._p[col_index],start_pva._v[col_index],start_pva._a[col_index]])

            endpva=np.array([end_pva._p[col_index],end_pva._v[col_index],end_pva._a[col_index]])
            
            Q_Matrix,Contraints=self.get_coefficient_matrixes(
                                                                waypoints_prt
                                                                ,n_poly
                                                                ,n_order
                                                                ,rank
                                                                ,time_vec
                                                                ,Q_weight
                                                                ,startpva
                                                                ,endpva
                                                                ,Q_weight
                                                            )
            
            P = sparse.csc_matrix(Q_Matrix)
            
            A = sparse.csc_matrix(Contraints.A)
            
            l = Contraints.l
            
            u = Contraints.u
            
            q = np.zeros([Q_Matrix.shape[0],1])
            
            prob = osqp.OSQP()
            
            prob.setup(P, q, A, l, u, alpha=1.0,verbose=False,max_iter=50000)
            
            res = prob.solve()

            if(self.status=='solved'):

                self.status=res.info.status

                if(self.status!='solved'):
                    
                    return False

            coefficients = res.x.reshape([n_poly,n_order+1])

            wp_id=[]
            
            for i in range(n_poly):         
                tt = np.arange(time_vec[i],time_vec[i+1],(time_vec[i+1]-time_vec[i])/100)
                xx,wp_id_ = self.calc_polys_points(coefficients[i],tt,n_order,i)             
                xyz_final= np.append(xyz_final,xx)
                wp_id.extend(wp_id_)
                T_draw = np.append(T_draw,tt)
            del prob
            
        return xyz_final,wp_id,T_draw,[res.x],n_poly,n_order

    def draw(self,xyz_final,waypoints_draw,T_draw):
        xyz_final = xyz_final.reshape([3,-1])
        T_draw = T_draw.reshape([3,-1])
        tt=T_draw[0]
        plt.figure(0)
        xx = xyz_final[0]
        yy = xyz_final[1]
        zz = xyz_final[2]
        plt.subplot(221)
        
        plt.plot(tt,xx,color = 'black')
        plt.title('X')
        plt.xlabel('Time')
        plt.ylabel('Value')
        plt.legend('X')
        plt.subplot(222)
        plt.title('Y')
        plt.plot(tt,yy,color = 'black')
        plt.xlabel('Time')
        plt.ylabel('Value')
        plt.legend('Y')
        plt.subplot(223)
        plt.title('Z')
        plt.plot(tt,zz,color = 'black')
        plt.xlabel('Time')
        plt.ylabel('Value')
        plt.legend('Z')
        
        fig = plt.figure(1)
        ax = fig.gca(projection='3d')
        ax.plot(xx,yy,zz,label='parametric curve ID:{}'.format(waypoints_draw[:,3][0]),color = 'black')
        ax.scatter(waypoints_draw[:,0],waypoints_draw[:,1],waypoints_draw[:,2],marker='*',color ='r')

        ax.legend() 

    '''
        分别计算每一列的数据
    '''
    def calc(self,waypoints,mode,start_pva,end_pva,draw_En):
        # xyz_final=np.array([0])
        # wp_id=np.array([0])
        # ret_coefficients=np.array([0])
        ret_n_poly=0
        ret_n_order=0
        if mode == "normal":
            for i in range(waypoints.shape[0]):
                fig = plt.figure(1)
                if draw_En == True :
                    xx=np.array(waypoints[:,0])
                    yy=np.array(waypoints[:,1])
                    zz=np.array(waypoints[:,2])
                    ax = fig.gca(projection='3d')
                    waypoints_draw = waypoints
                    ax.plot(xx,yy,zz,label='parametric curve ID:{}'.format(waypoints_draw[:,3][0]),color = 'black')
            ret = self.minimumsnap(waypoints
                                                                            ,self.n_order
                                                                            ,self.rank
                                                                            ,self.fly_speed
                                                                            ,start_pva
                                                                            ,end_pva
                                                                            ,self.Q_weight)
            if ret ==False:
                return False
            xyz_final_,wp_id_,T_draw,ret_coefficients_,ret_n_poly,ret_n_order=ret
            if(self.status=='solved'):
                if draw_En == True :
                    self.draw(xyz_final,waypoints_draw,T_draw)
                    plt.show()
            return xyz_final_,wp_id_,ret_coefficients_,ret_n_poly,ret_n_order

def handle_request(req):
    print('A request received.')
    success=False
    calc_result=''
    ret_wpx = []
    ret_wpy = []
    ret_wpz = []
    ret_wpvx = []
    ret_wpvy = []
    ret_wpvz = []
    ret_wpax = []
    ret_wpay = []
    ret_wpaz = []
    ret_id=[]
    ret_coefficients=[]
    ret_n_order=0
    ret_n_poly=0
    if (req.request == False):
        print('A reqest received.')
        return serviceResponse(success,'request == False.',ret_coefficients,ret_n_poly,ret_n_order,ret_wpx,ret_wpy,ret_wpz
        ,ret_wpvx
        ,ret_wpvy
        ,ret_wpvz
        ,ret_wpax
        ,ret_wpay
        ,ret_wpaz       
        ,ret_id)
    ms = minimum_snap()
    waypoints = np.array([req.waypointsx
                                        ,req.waypointsy
                                        ,req.waypointsz
                                        ,req.waypointsid]).T
    if req.mode not in ['normal','huge']:
        return serviceResponse(
                                success
                                ,'Mode synx wrong.'
                                ,ret_coefficients
                                ,ret_n_poly
                                ,ret_n_order
                                ,ret_wpx
                                ,ret_wpy
                                ,ret_wpz
                                ,ret_wpvx
                                ,ret_wpvy
                                ,ret_wpvz
                                ,ret_wpax
                                ,ret_wpay
                                ,ret_wpaz
                                ,ret_id
                                )
    if len(req.startpva)!=9 or len(req.endpva)!=9:
        print('num of pva wrong.')
        print(req.startpva)
        return serviceResponse(
                        success
                        ,'num of pva wrong.len(startpva):{},len(endpva):{}.Check PVA values'.format(len(req.startpva),len(req.endpva))
                        ,ret_coefficients
                        ,ret_n_poly
                        ,ret_n_order
                        ,ret_wpx
                        ,ret_wpy
                        ,ret_wpz
                        ,ret_wpvx
                        ,ret_wpvy
                        ,ret_wpvz
                        ,ret_wpax
                        ,ret_wpay
                        ,ret_wpaz   
                        ,ret_id)

    print("data parse right.")

    success = True

    startpva = minimum_snap.PVA(
                                np.array([req.startpva[0],req.startpva[1],req.startpva[2]])
                                ,np.array([req.startpva[3],req.startpva[4],req.startpva[5]])
                                ,np.array([req.startpva[6],req.startpva[7],req.startpva[8]])
                                )    
    print(startpva)
    endpva = minimum_snap.PVA(
                                np.array([req.endpva[0],req.endpva[1],req.endpva[2]])
                                ,np.array([req.endpva[3],req.endpva[4],req.endpva[5]])
                                ,np.array([req.endpva[6],req.endpva[7],req.endpva[8]])
                                )   
    print("data parse right22.")                
    ret=ms.calc(waypoints,req.mode,startpva,endpva,False)
    if ret == False:
        success=False
        return serviceResponse(
                        success
                        ,'primal infeasible.'
                        ,ret_coefficients
                        ,ret_n_poly
                        ,ret_n_order
                        ,ret_wpx
                        ,ret_wpy
                        ,ret_wpz
                        ,ret_wpvx
                        ,ret_wpvy
                        ,ret_wpvz
                        ,ret_wpax
                        ,ret_wpay
                        ,ret_wpaz   
                        ,ret_id)
    ret_xyz_final,ret_id,ret_coefficients,ret_n_poly,ret_n_order=ret
    del ms
    ret_xyz_final=ret_xyz_final.reshape([3,-1])
    ret_wpx = ret_xyz_final[0,:]
    ret_wpy = ret_xyz_final[1,:]
    ret_wpz = ret_xyz_final[2,:]
    ret_wpvx = np.array([])
    ret_wpvy = np.array([])
    ret_wpvz = np.array([])
    ret_wpax = np.array([])
    ret_wpay = np.array([])
    ret_wpaz = np.array([])
    print('data calc accomplished')
    return serviceResponse(success
                            ,'success'
                            ,ret_coefficients[0].tolist()
                            ,ret_n_poly
                            ,ret_n_order
                            ,ret_wpx.tolist()
                            ,ret_wpy.tolist()
                            ,ret_wpz.tolist()
                            ,ret_wpvx.tolist()
                            ,ret_wpvy.tolist()
                            ,ret_wpvz.tolist()
                            ,ret_wpax.tolist()
                            ,ret_wpay.tolist()
                            ,ret_wpaz.tolist()   
                            ,ret_id
                            )

rospy.init_node('minimumsnap_route')  #服务节点名
s = rospy.Service('minimumsnap_route/minimumsnap_calc_request',service,handle_request)
print('ros service inited')
print('waiting for request')
rospy.spin()

# ms = minimum_snap()

# file  = pd.read_csv('/home/az/matrixoutput2.csv',sep=';',header=None)

# waypoints= np.array(file)

# ms.calc(waypoints[0:5,:],"normal"
#                         ,minimum_snap.PVA(np.array([0,0,0]),np.array([0,0,0]),np.array([0,0,0]))
#                         ,minimum_snap.PVA(np.array([0,0,0]),np.array([0,0,0]),np.array([0,0,0]))
#                         ,False)
