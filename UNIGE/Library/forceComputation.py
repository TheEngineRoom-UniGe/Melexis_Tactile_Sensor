# -*- coding: utf-8 -*-
"""
Created on Fri Apr 29 08:13:41 2022

@author: tls
"""
#Data manipulation
import pandas as pd
import numpy as np

#ML libs (sklearn and tpot)
from sklearn.preprocessing import PolynomialFeatures
from sklearn.model_selection import train_test_split
from sklearn.linear_model import SGDRegressor
from sklearn.preprocessing import RobustScaler

class forceComputation():
    """
    Class that deals with the signal processing and the machine learning inference to extract the force from the magnetic values of the sensor
    
    The main function is InferenceForce that takes an array of magnetic values and outputs the force
    The initialization of the class consists in loading or generating the ML model and can take a few seconds.
    """
    
    def __init__(self,pathDataset=None,sfi=True):
        """
        Initialize the force computation algorithm. Use a dataset in argument
        """
        
        #Hardcode Parameters
        self.deg = 2
        self.SFI = sfi
        self.score = []
            

        print("Loading dataset")
        df_data = pd.read_csv(pathDataset)
        if ("U" in df_data.columns) and ("V" in df_data.columns) :
            df_data = ReferenceDF(df_data)  #Data needs to be referenced
        
        #Hardcode parameter of the regressor:
        SGDRegFx = SGDRegressor(max_iter=10000000, tol=1e-7,loss="huber",penalty="elasticnet",learning_rate = "invscaling",alpha=0.00039,fit_intercept=False,epsilon=0.0777)
        SGDRegFy = SGDRegressor(max_iter=10000000, tol=1e-7,loss="huber",penalty="elasticnet",learning_rate = "invscaling",alpha=0.00039,fit_intercept=False,epsilon=0.0777)
        SGDRegFz = SGDRegressor(max_iter=10000000, tol=1e-7,loss="huber",penalty="elasticnet",learning_rate = "invscaling",alpha=0.00039,fit_intercept=False,epsilon=0.0777)
        
        print("Preprocessing")
        df_train, df_test = train_test_split(df_data, test_size=0.2)
        df_train = Formatdf(df_train,deg=self.deg,stray=self.SFI)
        df_test_Bx = Formatdf_withsc(df_test,deg=self.deg,sc=df_train[2],stray=self.SFI)
        
        print("Training model")
        SGDRegFx.fit(df_train[0],df_train[1][:,0])
        SGDRegFy.fit(df_train[0],df_train[1][:,1])
        SGDRegFz.fit(df_train[0],df_train[1][:,2])
        print("Training complete")
        self.sc = df_train[2]
        self.sct = df_train[3]
        self.RegressorFx = SGDRegFx
        self.RegressorFy = SGDRegFy
        self.RegressorFz = SGDRegFz
        self.Regressor = [self.RegressorFx,self.RegressorFy,self.RegressorFz]
        print("Init done")
        
    
    
    def InferenceForceDf(self,Barray):
        """
        Computes the force from a dataframe (Barray) containing n lines of data with one magnetic feature per column
        [Bx00,Bz00,Bx01,Bz01,Bx10,Bz10,Bx11,Bz11]
        """
        #preprocessing of the incoming signal
        Bpost = FormatInference(Barray,deg=self.deg,sc=self.sc,stray=self.SFI)
        Fx = self.RegressorFx.predict(Bpost)
        Fy = self.RegressorFy.predict(Bpost)
        Fz = self.RegressorFz.predict(Bpost)
        Farr = np.array([Fx,Fy,Fz])
        Farr = np.swapaxes(Farr,0,1)
        Farr = self.sct.inverse_transform(Farr)
        return(Farr)
    
        
    def InferenceForce(self,B):
        """
        Computes the force from a single measurment, returns a 3 component array containing:[Fx,Fy,Fz]
        """
        Barray = np.reshape(B,(-1,8))
        Barray = pd.DataFrame(Barray,columns = ["Bx00","Bz00","Bx01","Bz01","Bx10","Bz10","Bx11","Bz11"])
        Farr = self.InferenceForceDf(Barray)
        return Farr[0]
     
        
def FormatInference(Barray,sc,deg=1,stray=False):
    """Signal processing and scaler operation to normalize it
    Barray is a dataFrame (pandas) that contains several lines of magnetic signal
    sc is the scaler used to normalize the features after the feature augmentation (created when training the ML model)
    deg is the degree of the polynomial augmentation
    stray is a boolean, True means that the features will be made stray-field robust (needs to be the same with the ML model)
    """
    Bpoly = featureIncmain(Barray,deg=deg,stray=stray)  #signal processing
    Bpoly = sc.transform(Bpoly)                         #normalization
    Bpoly[:,0]+=1                                       #first column filled with ones
    return Bpoly
    
def Formatdf(df,deg=1,stray=False):
    """ Function called when creating a new machine learning model, returns its parameters
    df is the dataframe that contains the training data
    deg is the degree of the polynomial augmentation
    stray is a boolean, True means that the features will be made stray-field robust
    """
    B = df[["Bx00","Bz00","Bx01","Bz01","Bx10","Bz10","Bx11","Bz11"]]
    Bpoly = featureIncmain(B,deg=deg,stray=stray)
    Res = df[["Fx","Fy","Fz"]]
    sc    = RobustScaler(with_centering=True,with_scaling=True,quantile_range=(25.0, 75.0),copy=True)
    sct   = RobustScaler(with_centering=True,with_scaling=True,quantile_range=(25.0, 75.0),copy=True)
    Bpoly = sc.fit_transform(Bpoly)
    Bpoly[:,0]+=1
    F_m = sct.fit_transform(Res)
    return Bpoly,F_m,sc,sct

def Formatdf_withsc(df,sc,deg=1,stray=False):
    """ Function called when testing the model on a training set
    df is the dataframe that contains the training data
    sc is the scaler previously used for the training that normalize the features
    deg is the degree of the polynomial augmentation
    stray is a boolean, True means that the features will be made stray-field robust
    """
    B = df[["Bx00","Bz00","Bx01","Bz01","Bx10","Bz10","Bx11","Bz11"]]
    Bpoly = featureIncmain(B,deg=deg,stray=stray)
    Bpoly = sc.transform(Bpoly)
    Bpoly[:,0]+=1
    return Bpoly


def featureIncmain(df,deg=2,stray=False):
    """
    Preprocessing of the data for better inference
    
    deg is the order of the polynomial augmentation
    stray is true if we want to use features robust against stray-field (mean field measured being removed, ie work in relative values)
    """
    X = df[["Bx00","Bz00","Bx01","Bz01","Bx10","Bz10","Bx11","Bz11"]].copy()
    if stray :
        BzMean = (X["Bz00"]+X["Bz01"]+X["Bz10"]+X["Bz11"])/4
        X["Bz00"] = X["Bz00"]-BzMean
        X["Bz01"] = X["Bz01"]-BzMean
        X["Bz10"] = X["Bz10"]-BzMean
        X["Bz11"] = X["Bz11"]-BzMean

        BxMean = (X["Bx00"]+X["Bx01"]+X["Bx10"]+X["Bx11"])/4
        X["Bx00"] = X["Bx00"]-BxMean
        X["Bx01"] = X["Bx01"]-BxMean
        X["Bx10"] = X["Bx10"]-BxMean
        X["Bx11"] = X["Bx11"]-BxMean
        
    #feature augmentation (norm of the magnetic field at the 4 sensing spots)
    Bnorm1 = np.sqrt(X["Bz00"]*X["Bz00"]+X["Bx00"]*X["Bx00"])
    Bnorm2 = np.sqrt(X["Bz01"]*X["Bz01"]+X["Bx01"]*X["Bx01"])
    Bnorm3 = np.sqrt(X["Bz10"]*X["Bz10"]+X["Bx10"]*X["Bx10"])
    Bnorm4 = np.sqrt(X["Bz11"]*X["Bz11"]+X["Bx11"]*X["Bx11"])

    X.insert(loc=8,column="Bnorm1",value = Bnorm1)
    X.insert(loc=9,column="Bnorm2",value = Bnorm2)
    X.insert(loc=10,column="Bnorm3",value = Bnorm3)
    X.insert(loc=11,column="Bnorm4",value = Bnorm4)

    poly = PolynomialFeatures(deg) #Polynomial augmentation from 12 to 91 features
    newX = poly.fit_transform(X)
    return newX

def ReferenceDF(df):
    """Correct the axes to put them in the reference of the sensor not the load cell"""
    dfout = df.copy()
    FxT = df["Fx"]*(np.cos(df["V"]*np.pi/180)) + df["Fz"]*(np.sin(df["V"]*np.pi/180))
    FyT = df["Fx"]*(np.sin(df["V"]*np.pi/180)*np.sin(df["U"]*np.pi/180)) + df["Fy"]*(np.cos(df["U"]*np.pi/180)) + df["Fz"]*(-1*np.cos(df["V"]*np.pi/180)*np.sin(df["U"]*np.pi/180))
    FzT = df["Fx"]*(-1*np.sin(df["V"]*np.pi/180)*np.cos(df["U"]*np.pi/180)) + df["Fy"]*(np.sin(df["U"]*np.pi/180)) + df["Fz"]*(np.cos(df["V"]*np.pi/180)*np.cos(df["U"]*np.pi/180))
    dfout["Fx"] = FxT
    dfout["Fy"] = FyT
    dfout["Fz"] = FzT
    return dfout
