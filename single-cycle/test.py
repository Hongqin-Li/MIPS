# This is used to help understanding the algorithm
# Can also used to check the answer
print ('Please input a seed')

seed = int(input())
a = 17
c = 3
m = 256

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

for x in lcg (m, a, c, seed):
    print (x)
