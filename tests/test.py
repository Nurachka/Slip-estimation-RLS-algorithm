def sum(a,b):

    c = a + b
    a = b
    return c, a


a = 10 
b = 20
c, a = sum(a, b)
print(a, b, c)
  