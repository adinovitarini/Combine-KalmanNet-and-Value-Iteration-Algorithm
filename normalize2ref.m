function Y = normalize2ref(X,R)
Y =((X-min(X))/(max(X)-min(X))*(max(R)-min(R)))+min(R);
end