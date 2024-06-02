#include "../main.h"
#include "Hand.h"

Hand::Hand()
{
    g<<0,0,-9.8;
    const char* finger1_urdf_path="../urdf/delto_gripper/delto_gripper_3f_finger1.urdf";
    const char* finger2_urdf_path="../urdf/delto_gripper/delto_gripper_3f_finger2.urdf";
    const char* finger3_urdf_path="../urdf/delto_gripper/delto_gripper_3f_finger3.urdf";
    pinocchio::urdf::buildModel(finger1_urdf_path, this->finger1);
    pinocchio::urdf::buildModel(finger2_urdf_path, this->finger2);
    pinocchio::urdf::buildModel(finger3_urdf_path, this->finger3);
    this->finger1.gravity.linear(this->g);
    this->finger2.gravity.linear(this->g);
    this->finger3.gravity.linear(this->g);
}

std::vector<MassMat> Hand::MassMatrix(std::vector<HJVec>  q_list)
{

    std::vector<MassMat>  Mmat_list;
    MassMat M1 = MassMat::Zero();
    MassMat M2 = MassMat::Zero();
    MassMat M3 = MassMat::Zero();


    pinocchio::Data data1(this->finger1);
    Eigen::VectorXd q1 = q_list.at(0);
    // Eigen::VectorXd v1 = pinocchio::randomConfiguration(this->finger1);
    // Eigen::VectorXd a1 = pinocchio::randomConfiguration(this->finger1);
    pinocchio::crba(this->finger1, data1, q1);
    data1.M.triangularView<Eigen::StrictlyLower>() =
        data1.M.transpose().triangularView<Eigen::StrictlyLower>();
    M1 = data1.M;


    pinocchio::Data data2(this->finger2);
    Eigen::VectorXd q2 = q_list.at(1);
    //Eigen::VectorXd v2 = pinocchio::randomConfiguration(this->finger2);
    //Eigen::VectorXd a2 = pinocchio::randomConfiguration(this->finger2);
    pinocchio::crba(this->finger2, data2, q2);
    data2.M.triangularView<Eigen::StrictlyLower>() =
        data2.M.transpose().triangularView<Eigen::StrictlyLower>();
    M2 = data2.M;


    pinocchio::Data data3(this->finger3);
    Eigen::VectorXd q3 = q_list.at(2);
    //Eigen::VectorXd v3 = pinocchio::randomConfiguration(this->finger3);
    //Eigen::VectorXd a3 = pinocchio::randomConfiguration(this->finger3);
    pinocchio::crba(this->finger3, data3, q3);
    data3.M.triangularView<Eigen::StrictlyLower>() =
        data3.M.transpose().triangularView<Eigen::StrictlyLower>();
    M3 = data3.M;

    Mmat_list.push_back(M1);
    Mmat_list.push_back(M2);
    Mmat_list.push_back(M3);
    return Mmat_list;
}


std::vector<HJVec> Hand::GravityForces(std::vector<HJVec>  q_list)
{

    std::vector<HJVec>  grav_list;
    HJVec grav1 = HJVec::Zero();
    HJVec grav2 = HJVec::Zero();
    HJVec grav3 = HJVec::Zero();


    pinocchio::Data data1(this->finger1);
    Eigen::VectorXd q1 = q_list.at(0);
    Eigen::VectorXd v1 = pinocchio::randomConfiguration(this->finger1);
    Eigen::VectorXd a1 = pinocchio::randomConfiguration(this->finger1);
    Eigen::VectorXd tau1 = pinocchio::rnea(this->finger1, data1, q1, v1 * 0, a1 * 0);    
    pinocchio::computeGeneralizedGravity(this->finger1, data1, q1);
    grav1 = data1.g;
    


    pinocchio::Data data2(this->finger2);
    Eigen::VectorXd q2 = q_list.at(1);
    Eigen::VectorXd v2 = pinocchio::randomConfiguration(this->finger2);
    Eigen::VectorXd a2 = pinocchio::randomConfiguration(this->finger2);
    Eigen::VectorXd tau2 = pinocchio::rnea(this->finger2, data2, q2, v2 * 0, a2 * 0);    
    pinocchio::computeGeneralizedGravity(this->finger2, data2, q2);
    grav2 = data2.g;


    pinocchio::Data data3(this->finger3);
    Eigen::VectorXd q3 = q_list.at(2);
    Eigen::VectorXd v3 = pinocchio::randomConfiguration(this->finger3);
    Eigen::VectorXd a3 = pinocchio::randomConfiguration(this->finger3);
    Eigen::VectorXd tau3 = pinocchio::rnea(this->finger3, data3, q3, v3 * 0, a3 * 0);    
    pinocchio::computeGeneralizedGravity(this->finger3, data3, q3);
    grav3 = data3.g;

    grav_list.push_back(grav1);
    grav_list.push_back(grav2);
    grav_list.push_back(grav3);
    return grav_list;
}


std::vector<MassMat> Hand::MassMatrixInverse(std::vector<HJVec>  q_list)
{

    std::vector<MassMat>  MmatInv_list;
    MassMat Minv1 = MassMat::Zero();
    MassMat Minv2 = MassMat::Zero();
    MassMat Minv3 = MassMat::Zero();


    pinocchio::Data data1(this->finger1);
    Eigen::VectorXd q1 = q_list.at(0);
    // Eigen::VectorXd v1 = pinocchio::randomConfiguration(this->finger1);
    // Eigen::VectorXd a1 = pinocchio::randomConfiguration(this->finger1);
    pinocchio::computeMinverse(this->finger1, data1, q1);
    data1.Minv.triangularView<Eigen::StrictlyLower>() =    data1.Minv.transpose().triangularView<Eigen::StrictlyLower>();
    Minv1 = data1.Minv;


    pinocchio::Data data2(this->finger2);
    Eigen::VectorXd q2 = q_list.at(1);
    //Eigen::VectorXd v2 = pinocchio::randomConfiguration(this->finger2);
    //Eigen::VectorXd a2 = pinocchio::randomConfiguration(this->finger2);
    pinocchio::computeMinverse(this->finger2, data2, q2);
    data2.Minv.triangularView<Eigen::StrictlyLower>() =    data2.Minv.transpose().triangularView<Eigen::StrictlyLower>();
    Minv2 = data2.Minv;


    pinocchio::Data data3(this->finger3);
    Eigen::VectorXd q3 = q_list.at(2);
    //Eigen::VectorXd v3 = pinocchio::randomConfiguration(this->finger3);
    //Eigen::VectorXd a3 = pinocchio::randomConfiguration(this->finger3);
    pinocchio::computeMinverse(this->finger3, data3, q3);
    data3.Minv.triangularView<Eigen::StrictlyLower>() =    data3.Minv.transpose().triangularView<Eigen::StrictlyLower>();
    Minv3 = data3.Minv;


    MmatInv_list.push_back(Minv1);
    MmatInv_list.push_back(Minv2);
    MmatInv_list.push_back(Minv3);
    return MmatInv_list;
}




Hand::Hand(const std::string hostname, const Poco::UInt16 Port)
{
    g<<0,0,-9.8;
    const char* finger1_urdf_path="../urdf/delto_gripper/delto_gripper_3f_finger1.urdf";
    const char* finger2_urdf_path="../urdf/delto_gripper/delto_gripper_3f_finger2.urdf";
    const char* finger3_urdf_path="../urdf/delto_gripper/delto_gripper_3f_finger3.urdf";
    pinocchio::urdf::buildModel(finger1_urdf_path, this->finger1);
    pinocchio::urdf::buildModel(finger2_urdf_path, this->finger2);
    pinocchio::urdf::buildModel(finger3_urdf_path, this->finger3);
    this->finger1.gravity.linear(this->g);
    this->finger2.gravity.linear(this->g);
    this->finger3.gravity.linear(this->g);

    cout << "Trying to connect Hand ..." << endl;
    this->sock.connect(SocketAddress(hostname, Port));
    Timespan timeout(1, 0);
    while (sock.poll(timeout, Poco::Net::Socket::SELECT_WRITE) == false)
    {
        cout << "Connecting to Hand ...";
    }
    cout << "Done " << endl;
    Slist1.resize(6, 4);
    Slist2.resize(6, 4);
    Slist3.resize(6, 4);
    Slist1 << 0, 0, -0.102200285690322, -0.10220099427322,
        0.0132, -0.0920995526335446, -9.35493493062234e-07, -2.26910594583265e-07,
        0, 0, -0.0448998350850704, -0.0882998350735015,
        0, -0.999999999866718, -1.63265283294212e-05, -1.63265283294212e-05,
        0, 0, 0.999999999733436, 0.999999999733436,
        1, 1.63267948958288e-05, 1.63270614557082e-05, 1.63270614557082e-05;

    Slist2 << -0.0196, -2.59809711552522e-07, 0.102199965680648, 0.102200674263083,
        -0.0132, 0.0920995526335249, 8.68696429485318e-07, 1.6011306788395e-07,
        0, 0.0195999913699982, -0.0448995278956131, -0.0882995278840442,
        0, 0.999999999866504, 1.56729385365157e-05, 1.56729385365157e-05,
        0, -6.5358979298913e-07, -0.999999999743893, -0.999999999743893,
        1, 1.63267948958288e-05, 1.63270614557082e-05, 1.63270614557082e-05;

    Slist3 << 0.0196, 3.80200648363967e-07, 0.102200605701457, 0.102201314283892,
        -0.0132, 0.0920995526335249, 8.68696429485318e-07, 1.6011306788395e-07,
        0, -0.0196000086247688, -0.0449001422748038, -0.0883001422632349,
        0, 0.999999999866504, 1.56729385365157e-05, 1.56729385365157e-05,
        0, -6.5358979298913e-07, -0.999999999743893, -0.999999999743893,
        1, 1.63267948958288e-05, 1.63270614557082e-05, 1.63270614557082e-05;

    SE3 F1_M01, F1_M12, F1_M23, F1_M34, F1_M45;
    SE3 F2_M01, F2_M12, F2_M23, F2_M34, F2_M45;
    SE3 F3_M01, F3_M12, F3_M23, F3_M34, F3_M45;

    F1_M01 << 1, 0, 0, -0.012834839,
        0, 1, 0, 1.10309e-07,
        0, 0, 1, 0.0833156,
        0, 0, 0, 1;

    F1_M12 << 1, 0, 0, -0.0214660233437796,
        0, 1, 0, 0.000519643691,
        0, 0, 1, 0.0123158426694348,
        0, 0, 0, 1;

    F1_M23 << 1, 0, 0, -0.021838769677848,
        0, 1, 0, -0.000801689982184922,
        0, 0, 1, 0.00647545495485294,
        0, 0, 0, 1;

    F1_M34 << 1, 0, 0, -0.0563150073208731,
        0, 1, 0, 0.000143039542122588,
        0, 0, 1, -0.000304442221129922,
        0, 0, 0, 1;

    F1_M45 << 1, 0, 0, -0.0317150451950281,
        0, 1, 0, 0.000137125146084302,
        0, 0, 1, 0.00961945104641627,
        0, 0, 0, 1;

    F2_M01 << 1, 0, 0, 0.012834838999928,
        0, 1, 0, -0.0196001100703345,
        0, 0, 1, 0.0833156,
        0, 0, 0, 1;

    F2_M12 << 1, 0, 0, 0.0214660230041412,
        0, 1, 0, -0.000519657720973645,
        0, 0, 1, 0.0123158426694348,
        0, 0, 0, 1;

    F2_M23 << 1, 0, 0, 0.0218387702018197,
        0, 1, 0, 0.000801675708587798,
        0, 0, 1, 0.00647545495485294,
        0, 0, 0, 1;

    F2_M34 << 1, 0, 0, 0.0563150072273719,
        0, 1, 0, -0.000143076349036542,
        0, 0, 1, -0.000304442221129922,
        0, 0, 0, 1;

    F2_M45 << 1, 0, 0, 0.0317150451053977,
        0, 1, 0, -0.000137145874714098,
        0, 0, 1, 0.00961945104641627,
        0, 0, 0, 1;

    F3_M01 << 1, 0, 0, 0.012834838999928,
        0, 1, 0, 0.0195998899296655,
        0, 0, 1, 0.0833156,
        0, 0, 0, 1;
    F3_M12 << 1, 0, 0, 0.0214660230041412,
        0, 1, 0, -0.000519657720973645,
        0, 0, 1, 0.0123158426694348,
        0, 0, 0, 1,
        F3_M23 << 1, 0, 0, 0.0218387702018197,
        0, 1, 0, 0.000801675708587798,
        0, 0, 1, 0.00647545495485294,
        0, 0, 0, 1;
    F3_M34 << 1, 0, 0, 0.0563150072273719,
        0, 1, 0, -0.000143076349036542,
        0, 0, 1, -0.000304442221129922,
        0, 0, 0, 1;
    F3_M45 << 1, 0, 0, 0.0317150451053977,
        0, 1, 0, -0.000137145874714098,
        0, 0, 1, 0.00961945104641627,
        0, 0, 0, 1;

    Mlist1.push_back(F1_M01);
    Mlist1.push_back(F1_M12);
    Mlist1.push_back(F1_M23);
    Mlist1.push_back(F1_M34);
    Mlist1.push_back(F1_M45);

    Mlist2.push_back(F2_M01);
    Mlist2.push_back(F2_M12);
    Mlist2.push_back(F2_M23);
    Mlist2.push_back(F2_M34);
    Mlist2.push_back(F2_M45);

    Mlist3.push_back(F3_M01);
    Mlist3.push_back(F3_M12);
    Mlist3.push_back(F3_M23);
    Mlist3.push_back(F3_M34);
    Mlist3.push_back(F3_M45);

    M1 << 0.99999999973344,-1.63267948914767e-05 ,1.63265283315973e-05 ,-0.144169684537529,
     1.63267948936527e-05, 0.999999999866718 ,-2.17606829907777e-15, -1.77129397803245e-06,
     -1.63265283294212e-05 ,2.66562055462299e-10 ,0.999999999866722 ,0.111421906449574,
      0, 0, 0, 1;
       M2 << -0.999999999743897, 1.56732050984841e-05, -1.63265283315938e-05, 0.144169684538658,
        -1.56732051007472e-05, -0.999999999877175 ,1.06730283451809e-11 ,-0.019598314306471,
        -1.63265283294212e-05 ,2.66562055462299e-10, 0.999999999866722, 0.111421906449574,
         0, 0, 0, 1;

    M3 << -0.999999999743897 ,1.56732050984841e-05 ,-1.63265283315938e-05 ,0.144169684538658 ,
    -1.56732051007472e-05 ,-0.999999999877175 ,1.06730283451809e-11 ,0.019601685693529 ,
    -1.63265283294212e-05 ,2.66562055462299e-10 ,0.999999999866722 ,0.111421906449574 ,
    0 ,0 ,0 ,1;


    Blist1 = Ad(TransInv(M1))*Slist1;
    Blist2 = Ad(TransInv(M2))*Slist2;
    Blist3 = Ad(TransInv(M3))*Slist3;
    Matrix6d G1, G2, G3, G4;

    G1 << 0.061 ,0 ,0 ,0, 0, 0, 0, 0.061 ,0 ,0 ,0 ,0 ,0 ,0 ,0.061 ,0 ,0 ,0 ,0 ,0 ,0 ,4.72123836457023e-05 ,-1.447752235444e-10, -2.457113229689e-12 ,0, 0, 0, -1.447752235444e-10 ,5.03009175408712e-05 ,-7.389531387076e-07 ,0 ,0, 0, -2.457113229689e-12 ,-7.389531387076e-07 ,7.14753389665343e-06;

    G2 << 0.025 ,0 ,0, 0, 0, 0, 0, 0.025, 0, 0, 0 ,0 ,0, 0, 0.025 ,0 ,0 ,0 ,0 ,0 ,0 ,4.7167810266729e-06 ,-2.07127519342e-07, -1.073118723205e-07 ,0, 0, 0, -2.07127519342e-07 ,3.5256647103825e-06 ,-8.9151864559e-07 ,0 ,0 ,0 ,-1.073118723205e-07 ,-8.9151864559e-07 ,3.6576208947354e-06;

    G3 << 0.051 ,0 ,0 ,0 ,0, 0, 0, 0.051, 0, 0, 0, 0, 0, 0, 0.051, 0, 0, 0, 0, 0, 0, 4.27841514510026e-06, -1.782923297582e-09 ,1.822873926966e-07 ,0, 0, 0, -1.782923297582e-09 ,1.56557569571823e-05 ,-1.432955890692e-07 ,0 ,0 ,0 ,1.822873926966e-07 ,-1.432955890692e-07 ,1.43848347999979e-05;

    G4 << 0.071 ,0 ,0 ,0 ,0 ,0 ,0 ,0.071 ,0, 0, 0, 0, 0, 0, 0.071, 0, 0, 0, 0, 0, 0, 5.94107756957603e-06, 2.8632711178e-09 ,1.5973559948348e-06 ,0 ,0 ,0 ,2.8632711178e-09 ,7.349570798348e-05 ,-1.9736673224e-07 ,0 ,0 ,0 ,1.5973559948348e-06 ,-1.9736673224e-07 ,7.2536659137776e-05;
    Glist.push_back(G1);
    Glist.push_back(G2);
    Glist.push_back(G3);
    Glist.push_back(G4);
}
void Hand::set_tau(const std::vector<HJVec> tau_list)
{

    HJVec tau1, tau2, tau3;
    tau1 = tau_list.at(0);
    tau2 = tau_list.at(1);
    tau3 = tau_list.at(2);

    unsigned char writeBuff[MAXBUF];
    writeBuff[0] = 0x03;
    writeBuff[1] = 0x28;

    writeBuff[2] = 0x01;
    writeBuff[3] = ((uint16_t)tau1(0) >> 8) & 0x00FF;
    writeBuff[4] = (uint16_t)tau1(0) & 0x00FF;

    writeBuff[5] = 0x02;
    writeBuff[6] = ((uint16_t)tau1(1) >> 8) & 0x00FF;
    writeBuff[7] = (uint16_t)tau1(1) & 0x00FF;

    writeBuff[8] = 0x03;
    writeBuff[9] = ((uint16_t)tau1(2) >> 8) & 0x00FF;
    writeBuff[10] = (uint16_t)tau1(2) & 0x00FF;

    writeBuff[11] = 0x04;
    writeBuff[12] = ((uint16_t)tau1(3) >> 8) & 0x00FF;
    writeBuff[13] = (uint16_t)tau1(3) & 0x00FF;

    writeBuff[14] = 0x05;
    writeBuff[15] = ((uint16_t)tau2(0) >> 8) & 0x00FF;
    writeBuff[16] = (uint16_t)tau2(0) & 0x00FF;

    writeBuff[17] = 0x06;
    writeBuff[18] = ((uint16_t)tau2(1) >> 8) & 0x00FF;
    writeBuff[19] = (uint16_t)tau2(1) & 0x00FF;

    writeBuff[20] = 0x07;
    writeBuff[21] = ((uint16_t)tau2(2) >> 8) & 0x00FF;
    writeBuff[22] = (uint16_t)tau2(2) & 0x00FF;

    writeBuff[23] = 0x08;
    writeBuff[24] = ((uint16_t)tau2(3) >> 8) & 0x00FF;
    writeBuff[25] = (uint16_t)tau2(3) & 0x00FF;

    writeBuff[26] = 0x09;
    writeBuff[27] = ((uint16_t)tau3(0) >> 8) & 0x00FF;
    writeBuff[28] = (uint16_t)tau3(0) & 0x00FF;

    writeBuff[29] = 0x0A;
    writeBuff[30] = ((uint16_t)tau3(1) >> 8) & 0x00FF;
    writeBuff[31] = (uint16_t)tau3(1) & 0x00FF;

    writeBuff[32] = 0x0B;
    writeBuff[33] = ((uint16_t)tau3(2) >> 8) & 0x00FF;
    writeBuff[34] = (uint16_t)tau3(2) & 0x00FF;

    writeBuff[35] = 0x0C;
    writeBuff[36] = ((uint16_t)tau3(3) >> 8) & 0x00FF;
    writeBuff[37] = (uint16_t)tau3(3) & 0x00FF;

    std::vector<uint8_t> crcData;
    for (int jj = 0; jj < 38; jj++)
    {
        crcData.push_back(writeBuff[jj]);
    }

    uint16_t crc = CRC16_ARC().calculate(crcData);

    writeBuff[39] = (crc >> 8) & 0x00FF;
    writeBuff[38] = (crc) & 0x00FF;
    this->sock.sendBytes(writeBuff, MAXBUF, 0);
    usleep(2000);
}

std::vector<HJVec> Hand::get_q()
{
    std::vector<HJVec> q_list;
    HJVec q1, q2, q3;

    HJVec current1, current2, current3;
    this->state = GETQ;
    unsigned char writeBuff[MAXBUF];
    writeBuff[0] = 0x01;
    writeBuff[1] = 0x05;
    writeBuff[2] = 0xEE;
    writeBuff[3] = 0xD2;
    writeBuff[4] = 0xDC;
    this->sock.sendBytes(writeBuff, MAXBUF, 0);
    usleep(2000);
    unsigned char receiveBuff[MAXBUF];
    this->sock.receiveBytes(receiveBuff, MAXBUF, 0);

    int16_t q1_temp[4];
    int16_t q2_temp[4];
    int16_t q3_temp[4];

    int16_t current1_temp[4];
    int16_t current2_temp[4];
    int16_t current3_temp[4];

    q1_temp[0] = ((receiveBuff[3] << 8)) | (receiveBuff[4]);         // rad
    current1_temp[0] = ((receiveBuff[5] << 8)) | (receiveBuff[6]);   // mA
    q1_temp[1] = ((receiveBuff[8] << 8)) | (receiveBuff[9]);         // rad
    current1_temp[1] = ((receiveBuff[10] << 8)) | (receiveBuff[11]); // mA
    q1_temp[2] = ((receiveBuff[13] << 8)) | (receiveBuff[14]);       // rad
    current1_temp[2] = ((receiveBuff[15] << 8)) | (receiveBuff[16]); // mA
    q1_temp[3] = ((receiveBuff[18] << 8)) | (receiveBuff[19]);       // rad
    current1_temp[3] = ((receiveBuff[20] << 8)) | (receiveBuff[21]); // mA

    q2_temp[0] = ((receiveBuff[20 + 3] << 8)) | (receiveBuff[20 + 4]);         // rad
    current2_temp[0] = ((receiveBuff[20 + 5] << 8)) | (receiveBuff[20 + 6]);   // mA
    q2_temp[1] = ((receiveBuff[20 + 8] << 8)) | (receiveBuff[20 + 9]);         // rad
    current2_temp[1] = ((receiveBuff[20 + 10] << 8)) | (receiveBuff[20 + 11]); // mA
    q2_temp[2] = ((receiveBuff[20 + 13] << 8)) | (receiveBuff[20 + 14]);       // rad
    current2_temp[2] = ((receiveBuff[20 + 15] << 8)) | (receiveBuff[20 + 16]); // mA
    q2_temp[3] = ((receiveBuff[20 + 18] << 8)) | (receiveBuff[20 + 19]);       // rad
    current2_temp[3] = ((receiveBuff[20 + 20] << 8)) | (receiveBuff[20 + 21]); // mA

    q3_temp[0] = ((receiveBuff[40 + 3] << 8)) | (receiveBuff[40 + 4]);         // rad
    current3_temp[0] = ((receiveBuff[40 + 5] << 8)) | (receiveBuff[40 + 6]);   // mA
    q3_temp[1] = ((receiveBuff[40 + 8] << 8)) | (receiveBuff[40 + 9]);         // rad
    current3_temp[1] = ((receiveBuff[40 + 10] << 8)) | (receiveBuff[40 + 11]); // mA
    q3_temp[2] = ((receiveBuff[40 + 13] << 8)) | (receiveBuff[40 + 14]);       // rad
    current3_temp[2] = ((receiveBuff[40 + 15] << 8)) | (receiveBuff[40 + 16]); // mA
    q3_temp[3] = ((receiveBuff[40 + 18] << 8)) | (receiveBuff[40 + 19]);       // rad
    current3_temp[3] = ((receiveBuff[40 + 20] << 8)) | (receiveBuff[40 + 21]); // mA


    for (int i = 0; i < 4; i++)
    {
        q1(i) = static_cast<float>(q1_temp[i]) * 0.1 / 180.0 * 3.141592;
        q2(i) = static_cast<float>(q2_temp[i]) * 0.1 / 180.0 * 3.141592;
        q3(i) = static_cast<float>(q3_temp[i]) * 0.1 / 180.0 * 3.141592;
        current1(i) = static_cast<float>(current1_temp[i]);
        current2(i) = static_cast<float>(current2_temp[i]);
        current3(i) = static_cast<float>(current3_temp[i]);
    }
    std::cout << "current1 : " << current1.transpose() << std::endl;
    std::cout << "current2 : " << current2.transpose() << std::endl;
    std::cout << "current3 : " << current3.transpose() << std::endl;

    q_list.push_back(q1);
    q_list.push_back(q2);
    q_list.push_back(q3);

    return q_list;
}