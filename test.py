
def lcg(modulus, a, c, seed):
    s = []
    while True:
        seed = (a * seed + c) % modulus
        if seed in s :
            print ('Total : ' + str(len(s)))
            break
        else :
            s.append(seed)
            yield seed

for i in range (200):
    for x in lcg (256, 17, 3, i):
        continue
        
