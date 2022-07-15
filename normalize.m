function Y = normalize(X)
Y = (X-min(X))/(max(X)-min(X));
end