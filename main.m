clear all;clc
disp('Case 1 : Distillation');
disp('Case 2 : Motor'); 
n = input('Enter a number that define your plant: ');
switch n 
    case 1
        disp('Distillation');
        distillate();
    case 2
        disp('Motor DC')
        motor();
end