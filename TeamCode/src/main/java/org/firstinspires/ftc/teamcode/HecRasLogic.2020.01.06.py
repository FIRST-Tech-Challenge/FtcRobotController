import HECRasFilterSTATION
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from sklearn import preprocessing


dfxy = HECRasFilterSTATION.HecRAS_Filter(r"C:\Users\JGemperline\OneDrive - DOI\Canal_Banks\Friant\Section5.txt")

print(dfxy.shape)

#this is a github test

#This is a branch test number 2

#Slice the data by section number 'i'
def SectionAnalize(i,dfxy):
    x = i
    y = i+2
    dfxy = dfxy.iloc[:,x:y].dropna()
    # print(dfxy)
    # xmin = int(dfxy.iloc[:,0].min())
    # xmax = int(dfxy.iloc[:,0].max())

    #split into left and right using min assuming min is invert of the channel
    # ymin = dfxy.iloc[:,1].idxmin()
    # ymax = dfxy.iloc[:,1].idxmax()

    #calculate rise and run from ymin loc and shifted over for better results
    # centerline = int(dfxy.iloc[:,0].count()/2)
    centerline = dfxy.iloc[:,0].max()/2
    dfxy['CL'] = abs(dfxy.iloc[:,0]-centerline)
    centerline = dfxy[dfxy.loc[:,'CL'] == dfxy.loc[:,'CL'].min()].index[0]

    leftxy = dfxy.iloc[0:centerline,:]
    rightxy = dfxy.iloc[centerline:,:]
    rightxy.reset_index(inplace = True,drop = True)
    # print(rightxy)
    ###Righside only
    try:
        rightxy['Δx'] = rightxy.iloc[:,0] - (rightxy.iloc[0,0]-100)
        rightxy['Δy'] = rightxy.iloc[:,1]  - (rightxy.iloc[0,1]+0)
        # rightxy = rightxy.iloc[1:,:]

        rightxy['Slope'] = rightxy['Δy']/rightxy['Δx']
        rightxy.fillna(0, inplace = True)
        rightxy.reset_index(inplace = True,drop = True)
        bankR = rightxy.Slope.idxmax()
        
        # print(bankR)
        
    except IndexError:
        bankR = 0

    #plot if needed
    # cmap = plt.cm.jet
    # norm = (rightxy.Slope-rightxy.Slope.min())/(rightxy.Slope.max()-rightxy.Slope.min())
    # plt.scatter(rightxy.iloc[:,0],rightxy.iloc[:,1],c=cmap(norm))
    # plt.plot(rightxy.iloc[bankR,0],rightxy.iloc[bankR,1],'bx')
    # plt.plot((rightxy.iloc[0,0]-100),(rightxy.iloc[0,1]+0),'bx')
    # plt.plot(rightxy.iloc[0,0],rightxy.iloc[0,1],'gx')
    # plt.show()

    ###Leftside only
    centerline = centerline-1
    try:
        leftxy['Δx'] = leftxy.iloc[:,0] - (leftxy.iloc[centerline,0]+100)
        leftxy['Δy'] = leftxy.iloc[:,1]  - (leftxy.iloc[centerline,1]+0)
        leftxy = leftxy.iloc[1:,:]

        leftxy['Slope'] = leftxy['Δy']/leftxy['Δx']
        leftxy.fillna(0, inplace = True)
        leftxy.reset_index(inplace = True,drop = True)
        bankL = leftxy.Slope.idxmin()
    except IndexError:
        bankL = 0
    #plot if needed
    # cmap = plt.cm.jet
    # norm = (leftxy.Slope-leftxy.Slope.max())/(leftxy.Slope.min()-leftxy.Slope.max())
    # plt.scatter(leftxy.iloc[:,0],leftxy.iloc[:,1],c=cmap(norm))
    # plt.plot(leftxy.iloc[bankL,0],leftxy.iloc[bankL,1],'rx')
    # plt.show()
    section = dfxy.columns[0][:-1] 
    banks = [section, leftxy.iloc[bankL,0],leftxy.iloc[bankL,1],rightxy.iloc[bankR,0],rightxy.iloc[bankR,1]]
    
    return(banks)

dfbanks = pd.DataFrame()
i = 0
while i <= 560:
    print(i)
    try:
        print(SectionAnalize(i,dfxy))
        df = pd.DataFrame(SectionAnalize(i,dfxy)).transpose()
        df.rename(columns = {0:'River_Station',1:'Left_Sta',2:'Left_Elev',3:'Right_Sta',4:'Right_Elev'}, inplace = True)
        df.reindex()
        dfbanks = pd.concat([dfbanks,df],axis = 0)   
    except IndexError:
        print('no data')
    i = i+2
dfbanks.to_excel(r"C:\Users\JGemperline\OneDrive - DOI\Canal_Banks\Friant\dfbanks.xlsx")
print(dfbanks)
'''
##### Adding more points
xsup = np.linspace(xmin,xmax,200)[1:]
xmod = dfxy.iloc[:,0].append(pd.Series(xsup)).reset_index(drop = True)
dfxy = pd.concat([dfxy,xmod],axis = 1, ignore_index=True)
dfxy.columns=['x','y','xsup']
dfxy.sort_values('xsup' ,inplace=True)
dfxy.x.interpolate(inplace = True)
dfxy.y.interpolate(inplace = True)

dfxy['Δx']= dfxy.x.shift(-1)-dfxy.x
dfxy['Δy']= dfxy.y.shift(-1)-dfxy.y
dfxy['s'] = dfxy.Δy/dfxy.Δx
#split in half to look at only one side of canal
#find the unmber of points in the data and find the half way count number
number = int(round(dfxy.count().iloc[0]/2,0))
print(number)

#create a data frame for left and right sides
dfleft = dfxy.iloc[0:number,:].reset_index()
dfright = dfxy.iloc[number:,:].reset_index()

#### Look ahead dist j for the right side
print('Right \n',dfright)
j = 55
i = 0
slope= []
while i < number-1:
    avgslope = dfright.ix[i:i+j,'s'].mean()
    slope.append(avgslope)
    i = i+1
dfright['sa'] = pd.DataFrame(slope)
dfright['Δsa'] = round(dfright.sa-dfright.sa.shift(1),6)

#look at avgslope diff in front and back and report diferance
i = 0
slopediff= []
while i < number-1:
    avgslopediff_front = dfright.Δsa[i:i+j].mean()
    if i > j:
        avgslopediff_back = dfright.Δsa[i-j:i].mean()
    elif i<=j:
        avgslopediff_back = dfright.Δsa[i]
    avgslopediff = (avgslopediff_front-avgslopediff_back*101)
    slopediff.append(avgslopediff)
    i = i+1

dfright['Δsa_corner'] = pd.DataFrame(slopediff)
y = dfright['y'].to_list()
x = dfright['x'].to_list()
s = dfright['s'].to_list()*2
point = dfright[dfright.sa<=0]
point = point[point.y==point.y.max()]
pointx = point.loc[:,'x'].to_list()
pointy = point.loc[:,'y'].to_list()
cmap = plt.cm.jet
norm = (dfright.sa-dfright.sa.min())/(dfright.sa.max()-dfright.sa.min())
plt.scatter(x, y, c=cmap(norm))
plt.plot(pointx,pointy,'rx')
plt.show()

#### Look ahead dist j for the left side ####

print('Left \n', dfleft)
j = 10
i = 0
slope= []
while i < number-1:
    avgslope = dfleft.ix[i:i+j,'s'].mean()
    slope.append(avgslope)
    i = i+1
dfleft['sa'] = pd.DataFrame(slope)
dfleft['Δsa'] = round(dfleft.sa-dfleft.sa.shift(1),6)
print(dfleft.iloc[50:60,:])
#look at avgslope diff in front and back and report diferance
i = 0
slopediff= []
while i < number-1:
    avgslopediff_front = dfleft.Δsa[i:i+j].mean()
    if i > j:
        avgslopediff_back = dfleft.Δsa[i-j:i].mean()
        
    elif i<=j:
        avgslopediff_back = dfleft.Δsa[i]


    avgslopediff = (avgslopediff_front-avgslopediff_back)
    slopediff.append(avgslopediff)

    i = i+1

dfleft['Δsa_corner'] = pd.DataFrame(slopediff)
print(dfleft)

y = dfleft['y'].to_list()
x = dfleft['x'].to_list()
s = dfleft['s'].to_list()*2
point = dfleft[dfleft.sa<=0]
point = point[point.y==point.y.max()]
print(point)
pointx = point.loc[:,'x'].to_list()
pointy = point.loc[:,'y'].to_list()
# print(dfleft)
cmap = plt.cm.jet
# norm = plt.Normalize(vmin=min(s), vmax=max(s))
norm = (dfleft.sa-dfleft.sa.min())/(dfleft.sa.max()-dfleft.sa.min())
plt.scatter(x, y, c=cmap(norm))
plt.plot(pointx,pointy,'rx')

plt.show()
'''