L1 = 10;
L2 = 10;
L3 = 10;
L5 = 20;
alpha1 = pi/2;
alpha2 = pi/2;

%%[theta d a alpha]
DH = [0 L1 0 0;
      0 0 0 alpha1;
      0 0 L2 0;
      0 0 L3 0;
      0 L5 0 -alpha2];

L(1) = Link(DH (1,1:4),'modified');
L(2) = Link(DH (2,1:4),'modified');
L(3) = Link(DH (3,1:4),'modified');
L(4) = Link(DH (4,1:4),'modified');
L(5) = Link(DH (5,1:4),'modified');

L(1).qlim = [0,pi];
L(2).qlim = [0,pi];
L(3).qlim = [0,pi];
L(4).qlim = [0,pi];
L(5).qlim = [0,pi];


RobotExam = SerialLink(L,'name','Robot5R');
q = [0 0 0 0 0];
RobotExam.plot(q,'workspace',[-50 50 -50 50 0 50])
RobotExam.teach;