#2018-10-18
#Scott Jin
p=[0.2, 0.2, 0.2, 0.2, 0.2] #Initial Belief where everything is unknown ==> Uniform Distribution
world=['green', 'red', 'red', 'green', 'green'] # The world as Ground Truth ===> A Map 
measurements = ['red', 'red']  # Series of Measurment coming in
motions = [1,1] #Information from Encoder about How far we moved 
#Measurement with uncertainty Encoded 
pHit = 0.6
pMiss = 0.2
#Movement parameter with uncertainty 
pExact = 0.8
pOvershoot = 0.1 #overshoot by 1 grid 
pUndershoot = 0.1 #undershoot by 1 grid 
def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    #Normalization to make Probablity add to 1
    for i in range(len(q)):
        q[i] = q[i] / s
    return q
def move(p, U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U) % len(p)] #Cyclic Shifts 
        s = s + pOvershoot * p[(i-U-1) % len(p)] # Overshooting 
        s = s + pUndershoot * p[(i-U+1) % len(p)] # UnderShooting
        q.append(s)
    return q
#Now with the move and sense we can update our belief and essentially do the localization in  this simplified world.
for k in range(len(measurements)):
    p = sense(p, measurements[k])
    p = move(p, motions[k]) 
print p         
