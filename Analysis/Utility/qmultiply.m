function qout = qmultiply(q1, q2)
% Computes the successive rotation of q2 then q1, as in A(q1)*A(q2)

qout = [XiMat(q2), q2] * q1; 