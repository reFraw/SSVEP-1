#include <robotics_function/main_robotics_function.h>
#include <cmath>

const double PI = 3.1415926535;

Eigen::MatrixXd defineJacoDH(const std::vector<double>& jointState) {

    // Structure of DH ros - a, alpha, d, theta

    int DH_PARAMS = 4;
    int N_JOINTS = jointState.size();

    Eigen::MatrixXd DH = Eigen::MatrixXd::Random(N_JOINTS, DH_PARAMS);

    // Set a
    DH(0,0) = 0.0;
    DH(1,0) = 0.0;
    DH(2,0) = 0.0;
    DH(3,0) = 0.0;
    DH(4,0) = 0.0;
    DH(5,0) = 0.0;
    DH(6,0) = 0.0;

    // Set alpha
    DH(0,1) = PI/2;
    DH(1,1) = PI/2;
    DH(2,1) = PI/2;
    DH(3,1) = PI/2;
    DH(4,1) = PI/2;
    DH(5,1) = PI/2;
    DH(6,1) = 0.0;

    // Set d
    DH(0,2) = 0.2755;
    DH(1,2) = 0.0;
    DH(2,2) = -0.41;
    DH(3,2) = -0.0098;
    DH(4,2) = -0.3111;
    DH(5,2) = 0.0;
    DH(6,2) = 0.2638;

    // Set theta
    DH(0,3) = jointState[0];
    DH(1,3) = jointState[1];
    DH(2,3) = jointState[2];
    DH(3,3) = jointState[3];
    DH(4,3) = jointState[4];
    DH(5,3) = jointState[5];
    DH(6,3) = jointState[6];

    return DH;

}

Eigen::Matrix4d homogeneous(const Eigen::Vector4d& rowDH) {

    double a = rowDH[0];
    double alpha = rowDH[1];
    double d = rowDH[2];
    double theta = rowDH[3];

    double ca = cos(alpha);
    double sa = sin(alpha); 
    double ct = cos(theta);
    double st = sin(theta);

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    A(0,0) = ct;
    A(1,0) = st;
    A(2,0) = 0.0;
    A(3,0) = 0.0;

    A(0,1) = -st*ca;
    A(1,1) = ct*ca;
    A(2,1) = sa;
    A(3,1) = 0.0;

    A(0,2) = st*sa;
    A(1,2) = -ct*sa;
    A(2,2) = ca;
    A(3,2) = 0.0;

    A(0,3) = a*ct;
    A(1,3) = a*st;
    A(2,3) = d;
    A(3,3) = 1.0;

    return A;
}

Eigen::Matrix4d directKinematics(const std::vector<double>& jointState, const int& nJoints) {

    Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
    Eigen::MatrixXd DH = defineJacoDH(jointState);

    Eigen::Vector4d row = DH.row(0);
    T = homogeneous(row);

    for(int i = 1; i < nJoints; i++) {
        row = DH.row(i);
        T = T * homogeneous(row);
    } 

    return T;
}

Eigen::MatrixXd jacobian4jaco(const std::vector<double>& jointState) {

    int N_JOINTS = jointState.size();
    int N_ROWS = 6;

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(N_ROWS, N_JOINTS);

    Eigen::Vector3d p0 = {0.0, 0.0, 0.0};
    Eigen::Vector3d z0 = {0.0, 0.0, 1.0};

    Eigen::Matrix4d T = directKinematics(jointState, N_JOINTS);
    Eigen::Vector3d pE = {T(0,3), T(1,3), T(2,3)};

    J.col(0).head(3) = z0.cross(pE-p0);
    J.col(0).tail(3) = z0;

    for(int i = 1; i < N_JOINTS; i++) {

        T = directKinematics(jointState, i);

        Eigen::Vector3d pEi = {T(0,3), T(1,3), T(2,3)};
        Eigen::Vector3d zi = {T(0,2), T(1,2), T(2,2)};

        J.col(i).head(3) = zi.cross(pE-pEi);
        J.col(i).tail(3) = zi;

    }

    return J;

}

double raisedCos(const double& sigma, const double& lambda, const double& threshold) {

    double s = std::abs(sigma);

    if (s < threshold) {
        double temp = (s/threshold)*PI;
        return lambda * (0.5*cos(temp) + 0.5);
    } else {
        return 0;
    }
}

Eigen::MatrixXd pinvDamped(const Eigen::MatrixXd& J) {

   Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

   double threshold = 0.01;
   double lambda = 0.01;

   Eigen::MatrixXd U = svd.matrixU();
   Eigen::MatrixXd S = svd.singularValues();
   Eigen::MatrixXd V = svd.matrixV();

   int N_ROWS = J.rows();
   int N_COLS = J.cols();

   Eigen::MatrixXd invS = Eigen::MatrixXd::Zero(N_COLS, N_ROWS); 

   for(int i = 0; i < std::min(N_COLS, N_ROWS); i++) {
    double eigVal = S(i);
    double p = raisedCos(eigVal, lambda, threshold);
    invS(i,i) = eigVal / (p + std::pow(eigVal, 2));
   }

   return V.leftCols(N_COLS)*invS*U.transpose();
}

Eigen::Vector3d quaternionError(const Eigen::Quaterniond& QD, const Eigen::Quaterniond& QE) {

    double etaE = QE.w();
    double etaD = QD.w();

    Eigen::Vector3d epsE = {QE.x(), QE.y(), QE.z()};
    Eigen::Vector3d epsD = {QD.x(), QD.y(), QD.z()};

    Eigen::Vector3d eO = etaE*epsD - etaD*epsE - epsD.cross(epsE);

    double deltaEta = etaE*etaD + epsE.dot(epsD);

    if (deltaEta < 0) {
        eO = -eO;
    }

    return eO;

}

Eigen::Quaterniond rot2quat(const Eigen::Matrix3d& R) {

    double r11 = R(0, 0);
    double r12 = R(0, 1);
    double r13 = R(0, 2);

    double r21 = R(1, 0);
    double r22 = R(1, 1);
    double r23 = R(1, 2);

    double r31 = R(2, 0);
    double r32 = R(2, 1);
    double r33 = R(2, 2);

    double w = std::sqrt(r11 + r22 + r33 + 1) / 2;
    double x = 0.5*std::sqrt(r11 - r22 - r33 + 1);
    double y = 0.5*std::sqrt(r22 - r33 - r11 + 1);
    double z = 0.5*std::sqrt(r33 - r11 - r22 + 1);

    if (r32 - r23 < 0) {x = -x;}
    if (r13 - r31 < 0) {y = -y;}
    if (r21 - r12 < 0) {z = -z;}

    Eigen::Quaterniond Q(w, x, y, z);

    if (std::isnan(Q.w())){
        Q.w() = 0.0;
    }
    if (std::isnan(Q.x())){
        Q.x() = 0.0;
    }
    if (std::isnan(Q.y())){
        Q.y() = 0.0;
    }
    if (std::isnan(Q.z())){
        Q.z() = 0.0;
    }

    Q.normalize();

    return Q;
}

Eigen::Matrix3d quat2rot(const Eigen::Quaterniond& Q) {

    double eta = Q.w();
    double x = Q.x();
    double y = Q.y();
    double z = Q.z();

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();

    R(0,0) = 2*( std::pow(eta,2) + std::pow(x,2) ) - 1;
    R(0,1) = 2*(x*y - eta*z);
    R(0,2) = 2*(x*z + eta*y);

    R(1,0) = 2*(x*y +eta*z);
    R(1,1) = 2*( std::pow(eta,2) + std::pow(y,2) ) - 1;
    R(1,2) = 2*(y*z - eta*x);

    R(2,0) = 2*(x*z - eta*y);
    R(2,1) = 2*(y*z + eta*x);
    R(2,2) = 2*( std::pow(eta,2) + std::pow(z,2) ) - 1;

    return R;

}

Eigen::AngleAxisd rot2angleaxis(const Eigen::Matrix3d& R) {

    double r11 = R(0,0);
    double r12 = R(0,1);
    double r13 = R(0,2);

    double r21 = R(1,0);
    double r22 = R(1,1);
    double r23 = R(1,2);

    double r31 = R(2,0);
    double r32 = R(2,1);
    double r33 = R(2,2);

    double angle = std::acos((r11+r22+r33-1)/2);
    double x = r32 - r23;
    double y = r13 - r31;
    double z = r21 - r12;

    Eigen::Vector3d axis = {x, y, z};
    axis = (1/(2*sin(angle)))*axis;
    axis.normalize();


    Eigen::AngleAxisd AA(angle, axis);

    return AA;
}

Eigen::Matrix3d angleaxis2rot(const Eigen::AngleAxisd& AA) {

    Eigen::Vector3d axis = AA.axis();
    double theta = AA.angle();

    double x = axis[0];
    double y = axis[1];
    double z = axis[2];
    double s = sin(theta);
    double c = cos(theta);

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();

    R(0,0) = std::pow(x,2)*(1-c) + c;
    R(0,1) = x*y*(1-c) - z*s;
    R(0,2) = x*z*(1-c) + y*s;

    R(1,0) = x*y*(1-c) + z*s;
    R(1,1) = std::pow(y,2)*(1-c) + c;
    R(1,2) = y*z*(1-c) - x*s;

    R(2,0) = x*z*(1-c) - y*s;
    R(2,1) = y*z*(1-c) + x*s;
    R(2,2) = std::pow(z,2)*(1-c) + c;

    return R;
}

Eigen::Matrix3d rpy2rot(const double& roll, const double& pitch, const double& yaw) {

    double sf = sin(roll);
    double cf = cos(roll);

    double st = sin(pitch);
    double ct = cos(pitch);

    double sp = sin(yaw);
    double cp = cos(yaw);

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();

    R(0,0) = cf*ct;
    R(0,1) = cf*st*sp - sf*cp;
    R(0,2) = cf*st*cp + sf*sp;
    
    R(1,0) = sf*ct;
    R(1,1) = sf*st*sp + cf*cp;
    R(1,2) = sf*st*cp - cf*sp;

    R(2,0) = -st;
    R(2,1) = ct*sp;
    R(2,2) = ct*cp;

    return R;
}

double computePosition(const double& qi, const double& qf, const double& qddot, const double& tf, const double& tc, const double& t) {

    double pos;

    if (t >= 0 && t <= tc) {
        pos = qi + 0.5*qddot*std::pow(t,2);
    }
    else if (t > tc && t <= tf-tc) {
        pos = qi + qddot*tc*(t-0.5*tc);
    }
    else if (t > (tf-tc) && t <= tf) {
        pos = qf - 0.5*qddot*std::pow((tf-t),2);
    }
    else {
        pos = qf;
    }
    return pos;

}

double computeVelocity(const double& qddot, const double& tf, const double& tc, const double& t) {

    double vel;

    if (t >= 0 && t <= tc) {
        vel = qddot*t;
    }
    else if (t > tc && t <= tf-tc) {
        vel = qddot*tc;
    }
    else if (t > (tf-tc) && t <= tf) {
        vel = qddot*(tf-t);
    }
    else {
        vel = 0;
    }
    return vel;

}

double computeAcceleration(const double& qddot, const double& tf, const double& tc, const double& t) {

    double acc;

    if (t >= 0 && t <= tc) {
        acc = qddot;
    }
    else if (t > tc && t <= tf-tc) {
        acc = 0;
    }
    else if (t > (tf-tc) && t <= tf) {
        acc = -qddot;
    }
    else {
        acc = 0;
    }
    return acc;

}

Eigen::Vector3d trapezoidalProfile(const double& qi, const double& qf, const double& qdot, const double& tf, const double& t) {

    double delta = qf - qi;
    double checkTerm = std::abs(qf-qi) / tf;

    double cv;
    if (std::abs(qdot) > checkTerm && std::abs(qdot) <= 2*checkTerm) {
        cv = qdot;
    } else {
        cv = 1.5*checkTerm;
    }
    if (delta < 0) {
        cv = -cv;
    }

    double commonTerm = qi - qf + cv*tf;

    double tc = commonTerm / cv;
    double qddot = std::pow(cv,2) / commonTerm;

    double s = computePosition(qi, qf, qddot, tf, tc, t);
    double ds = computeVelocity(qddot, tf, tc, t);
    double dds = computeAcceleration(qddot, tf, tc, t);

    Eigen::Vector3d trapz = {s, ds, dds};

    return trapz;
}

Eigen::Vector3d planSegment(const Eigen::Vector3d& pi, const Eigen::Vector3d& pf, const double& tf, const double& velocity, const double& t) {

    Eigen::Vector3d point;

    if (pi == pf) {

        point = pi;
    }
    else {
        Eigen::Vector3d path = pf - pi;
        double pathLength = path.norm();

        Eigen::Vector3d s = trapezoidalProfile(0, pathLength, velocity, tf, t);

        point = pi + (s[0]/pathLength)*path;
    }

    return point;
}

std::vector<Eigen::Vector3d> planSegmentWithVelocity(const Eigen::Vector3d& pi, const Eigen::Vector3d& pf, const double& tf, const double& velocity, const double& t) {

    Eigen::Vector3d point;
    Eigen::Vector3d dpoint;

    std::vector<Eigen::Vector3d> res(2);

    if (pi == pf) {

        point = pi;
        dpoint[0] = 0.0;
        dpoint[1] = 0.0;
        dpoint[2] = 0.0;
    }
    else {
        Eigen::Vector3d path = pf - pi;
        double pathLength = path.norm();

        Eigen::Vector3d s = trapezoidalProfile(0, pathLength, velocity, tf, t);

        point = pi + (s[0]/pathLength)*path;
        dpoint = s[1]*((1/pathLength)*path);
    }

    res[0] = point;
    res[1] = dpoint;

    return res;

}

Eigen::Quaterniond planOrientation(const Eigen::Quaterniond& QI, const Eigen::Quaterniond& QF, const double& tf, const double& velocity, const double& t) {

    Eigen::Quaterniond desiredQuaternion;

    if (QI.w() == QF.w() && QI.x() == QF.x() && QI.y() == QF.y() && QI.z() == QF.z()) {

        desiredQuaternion = QI;

    }

    else {

        Eigen::Matrix3d Ri = quat2rot(QI);
        Eigen::Matrix3d Rf = quat2rot(QF);

        Eigen::Matrix3d Rit = Ri.transpose();
        Eigen::Matrix3d Rif = Rit*Rf;

        Eigen::AngleAxisd AA = rot2angleaxis(Rif);

        double finalTheta = AA.angle();
        Eigen::Vector3d rotationAxis = AA.axis();
        rotationAxis.normalize();

        Eigen::Vector3d th = trapezoidalProfile(0, finalTheta, velocity, tf, t);
        Eigen::AngleAxisd NA;
        NA.angle() = th[0];
        NA.axis() = rotationAxis;
        Eigen::Matrix3d instantRotation = NA.toRotationMatrix();
        Eigen::Matrix3d Rr = Ri*instantRotation;

        desiredQuaternion = rot2quat(Rr);
    }

    return desiredQuaternion;
}

std::vector<Eigen::Quaterniond> planOrientationWithVelocity(const Eigen::Quaterniond& QI, const double& tf, const double& velocity, const double& t, Eigen::Vector3d rotAxis, double rotAngle) {

    Eigen::Quaterniond desiredQuaternion;
    Eigen::Quaterniond desiredAngularVelocity;

    std::vector<Eigen::Quaterniond> res(2);

    Eigen::Matrix3d Ri = QI.toRotationMatrix();

    Eigen::Vector3d th = trapezoidalProfile(0, rotAngle, velocity, tf, t);
    
    Eigen::AngleAxisd NA(th[0], rotAxis);
    Eigen::Matrix3d  Rr = Ri*NA.toRotationMatrix();
    desiredQuaternion = Eigen::Quaterniond(Rr);

    Eigen::Vector3d omegai = th[1]*rotAxis;
    Eigen::Vector3d omegae = Ri*omegai;

    desiredAngularVelocity.x() = omegae[0];
    desiredAngularVelocity.y() = omegae[1];
    desiredAngularVelocity.z() = omegae[2];

    res[0] = desiredQuaternion;
    res[1] = desiredAngularVelocity;

    return res;


    //////////////////////////////////////////////////////////////////////////////////////////

    // Eigen::Matrix3d Ri = QI.toRotationMatrix();
    // Eigen::Matrix3d Rf = QF.toRotationMatrix();

    // Eigen::Matrix3d Rit = Ri.transpose();
    // Eigen::Matrix3d Rif = Rit*Rf;

    // Eigen::AngleAxisd AA = rot2angleaxis(Rif);

    // double finalTheta = AA.angle();
    // Eigen::Vector3d rotationAxis = AA.axis();
    // rotationAxis.normalize();

    // Eigen::Vector3d th = trapezoidalProfile(0, finalTheta, velocity, tf, t);
    // Eigen::AngleAxisd NA;
    // NA.angle() = th[0];
    // NA.axis() = rotationAxis;
    // Eigen::Matrix3d instantRotation = angleaxis2rot(NA);
    // Eigen::Matrix3d Rr = Ri*instantRotation;

    // Eigen::Vector3d omegai = th[1]*rotationAxis;
    // Eigen::Vector3d omega_e = Ri*omegai;

    // desiredAngularVelocity.x() = omega_e[0];
    // desiredAngularVelocity.y() = omega_e[1];
    // desiredAngularVelocity.z() = omega_e[2];
    // desiredQuaternion = Eigen::Quaterniond(Rr);


    // res[0] = desiredQuaternion;
    // res[1] = desiredAngularVelocity;

    // return res;    

}

std::vector<double> convertToJaco2(const std::vector<double>& jointState) {

    int N_JOINTS = jointState.size();

    std::vector<double> joint_jaco(N_JOINTS);

    joint_jaco[0] = -jointState[0];
    joint_jaco[1] = jointState[1] + PI;
    joint_jaco[2] = jointState[2];
    joint_jaco[3] = jointState[3];
    joint_jaco[4] = jointState[4];
    joint_jaco[5] = jointState[5] + PI;
    joint_jaco[6] = -jointState[6] - PI/2; // AGGIUNTA DI -PI/2 SENZA SENSO

    return joint_jaco;
}

std::vector<double> convertToDH(const std::vector<double>& jointState) {

    int N_JOINTS = jointState.size();

    std::vector<double> joint_DH(N_JOINTS);

    joint_DH[0] = -jointState[0];
    joint_DH[1] = jointState[1] - PI;
    joint_DH[2] = jointState[2];
    joint_DH[3] = jointState[3];
    joint_DH[4] = jointState[4];
    joint_DH[5] = jointState[5] - PI;
    joint_DH[6] = -jointState[6] + PI/2;

    return joint_DH;
}

std::vector<std::vector<double>> applyIK(const Eigen::Vector3d& xd, const Eigen::Quaterniond& qd, const std::vector<double>& jointState, const double& Gain) {

    std::vector<std::vector<double>> results(2);

    std::vector<double> jointVelocities(jointState.size());
    std::vector<double> error(6);

    Eigen::Matrix4d T = directKinematics(jointState, jointState.size());

    Eigen::Vector3d xe = T.block<3,1>(0,3);
    Eigen::Quaterniond qe = rot2quat(T.block<3,3>(0,0));

    Eigen::Vector3d ep = xd - xe;
    Eigen::Vector3d eo = quaternionError(qd, qe);

    Eigen::VectorXd err(ep.size() + eo.size());
    err << ep, eo;

    Eigen::MatrixXd J = jacobian4jaco(jointState);
    Eigen::MatrixXd invJ = pinvDamped(J);

    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6, 6);
    K *= Gain;

    Eigen::VectorXd dq = invJ*K*err;

    for (int i = 0; i < dq.size(); i++) {
        jointVelocities[i] = dq[i];
    }
    for (int i = 0; i < error.size(); i++) {
        error[i] = err[i]; 
    }

    results[0] = jointVelocities;
    results[1] = error;

    return results;

}

std::vector<std::vector<double>> applyIK2(const Eigen::Vector3d& xd, const Eigen::Quaterniond& qd, const Eigen::Vector3d& vd, const Eigen::Vector3d& omegad, const std::vector<double>& jointState, const double& Gain) {

    std::vector<std::vector<double>> results(2);

    std::vector<double> jointVelocities(jointState.size());
    std::vector<double> error(6);

    Eigen::Matrix4d T = directKinematics(jointState, jointState.size());

    Eigen::Vector3d xe = T.block<3,1>(0,3);
    Eigen::Quaterniond qe = Eigen::Quaterniond(T.block<3,3>(0,0));

    Eigen::Vector3d ep = xd - xe;
    Eigen::Vector3d eo = quaternionError(qd, qe);

    Eigen::VectorXd err(ep.size() + eo.size());
    err << ep, eo;

    Eigen::VectorXd xdotd(vd.size() + omegad.size());
    xdotd << vd, omegad;

    Eigen::MatrixXd J = jacobian4jaco(jointState);
    Eigen::MatrixXd invJ = pinvDamped(J);

    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6, 6);
    K *= Gain;

    Eigen::VectorXd dq = invJ*(xdotd + K*err);

    for (int i = 0; i < dq.size(); i++) {
        jointVelocities[i] = dq[i];
    }
    for (int i = 0; i < error.size(); i++) {
        error[i] = err[i]; 
    }

    results[0] = jointVelocities;
    results[1] = error;

    return results;

}

double computeManipulabity(const std::vector<double>& jointConfig) {

    Eigen::MatrixXd J = jacobian4jaco(jointConfig);
    Eigen::MatrixXd Jtransp = J.transpose();
    Eigen::MatrixXd M = J*Jtransp;

    double w = std::sqrt(M.determinant());

    return w;

}

Eigen::VectorXd computeNullVelocity(const std::vector<double>& jointConfig, const double& h, const double& k0) {

    int nJoints = jointConfig.size();

    Eigen::VectorXd dwdq(nJoints);

    for(int i = 0; i < nJoints; i++) {

        std::vector<double> qPlus = jointConfig;
        std::vector<double> qMinus = jointConfig;

        qPlus[i] = qPlus[i] + h;
        qMinus[i] = qMinus[i] - h;

        double wPlus = computeManipulabity(qPlus);
        double wMinus = computeManipulabity(qMinus);

        dwdq[i] = k0*(wPlus - wMinus)/(2*h);

    }

    return dwdq;

}

std::vector<std::vector<double>> applyIK2withNull(const Eigen::Vector3d& xd, const Eigen::Quaterniond& qd, const Eigen::Vector3d& vd, const Eigen::Vector3d& omegad, const std::vector<double>& jointState, const double& Gain, const Eigen::VectorXd& nullVelocity) {

    std::vector<std::vector<double>> results(2);

    std::vector<double> jointVelocities(jointState.size());
    std::vector<double> error(6);

    Eigen::Matrix4d T = directKinematics(jointState, jointState.size());

    Eigen::Vector3d xe = T.block<3,1>(0,3);
    Eigen::Quaterniond qe = Eigen::Quaterniond(T.block<3,3>(0,0));

    Eigen::Vector3d ep = xd - xe;
    Eigen::Vector3d eo = quaternionError(qd, qe);

    Eigen::VectorXd err(ep.size() + eo.size());
    err << ep, eo;

    Eigen::VectorXd xdotd(vd.size() + omegad.size());
    xdotd << vd, omegad;

    Eigen::MatrixXd J = jacobian4jaco(jointState);
    Eigen::MatrixXd invJ = pinvDamped(J);

    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6, 6);
    K *= Gain;

    Eigen::MatrixXd nullP = Eigen::MatrixXd::Identity(7,7) - invJ*J;

    Eigen::VectorXd dq = invJ*(xdotd + K*err) + nullP*nullVelocity;

    for (int i = 0; i < dq.size(); i++) {
        jointVelocities[i] = dq[i];
    }
    for (int i = 0; i < error.size(); i++) {
        error[i] = err[i]; 
    }

    results[0] = jointVelocities;
    results[1] = error;

    return results;

}






























































































































