function [x_hat_next,y_hat,KG] = KalmanNet(x_next_nw,y,A,C)
%%  Normalize state trajectories
delta_x = [x_next_nw;y];
for i =1:size(delta_x,1)
    delta_x(:,i)= normalize(delta_x(:,i));
    if i<size(delta_x,2)
    x_next_nw(:,i) = normalize(x_next_nw(:,i));
    end
end
%%  Propagate 
x_hat_kf = A*x_next_nw;
y_hat_kf = C*x_hat_kf;
X = delta_x;
Y = y_hat_kf;
numFeatures = size(X,1);
numResponses = size(Y,1);
numHiddenUnits = 10;
layers = [ ...
    sequenceInputLayer(numFeatures)
    lstmLayer(numHiddenUnits,'StateActivationFunction','tanh')
    fullyConnectedLayer(numResponses)
    regressionLayer];
options = trainingOptions('adam', ...
    'MaxEpochs',1000, ...
    'GradientThreshold',1, ...
    'InitialLearnRate',0.5, ...
    'LearnRateSchedule','piecewise', ...
    'LearnRateDropPeriod',125, ...
    'LearnRateDropFactor',0.2, ...
    'Verbose',0, ...
    'Plots','training-progress');
%% Train LSTM Net
net = trainNetwork(X,Y,layers,options);
%% Predict 
net = predictAndUpdateState(net,X);
[~,KG] = predictAndUpdateState(net,X);
x_hat_next = (KG*Y')+x_hat_kf;
y_hat = C*x_hat_next;
end