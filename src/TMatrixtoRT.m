function [R,T] = TMatrixtoRT(t)

R = [t(1,1),t(1,2),t(1,3);
    t(2,1),t(2,2),t(2,3);
    t(3,1),t(3,2),t(3,3)];

T = [t(1,4);
    t(2,4);
    t(3,4)];


end

