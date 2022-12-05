
#include "NetworkServerApp.h"
//#include "inet/networklayer/ipv4/IPv4Datagram.h"
//#include "inet/networklayer/contract/ipv4/IPv4ControlInfo.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/transportlayer/common/L4PortTag_m.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/ModuleAccess.h"
#include "inet/applications/base/ApplicationPacket_m.h"

#include "inet/networklayer/common/L3Tools.h"
#include "inet/networklayer/ipv4/Ipv4Header_m.h"
#include <iostream>
#include <fstream>


namespace flora {

Define_Module(NetworkServerApp);

void NetworkServerApp::initialize(int stage)
{
    if (stage == 0) {
        ASSERT(recvdPackets.size()==0);
        LoRa_ServerPacketReceived = registerSignal("LoRa_ServerPacketReceived");
        localPort = par("localPort");
        destPort = par("destPort");
        adrMethod = par("adrMethod").stdstringValue();
        std::cout<<"ADR Method is: "<< adrMethod<<"\n";
    } else if (stage == INITSTAGE_APPLICATION_LAYER) {
        startUDP();
        getSimulation()->getSystemModule()->subscribe("LoRa_AppPacketSent", this);
        evaluateADRinServer = par("evaluateADRinServer");
        adrDeviceMargin = par("adrDeviceMargin");
        receivedRSSI.setName("Received RSSI");
        totalReceivedPackets = 0;
        totalReceivedUniquePackets = 0;


        for(int i=0;i<6;i++){
            counterUniqueReceivedPacketsPerSF[i] = 0;
            counterOfSentPacketsFromNodesPerSF[i] = 0;
        }
    }
}


void NetworkServerApp::startUDP()
{
    socket.setOutputGate(gate("socketOut"));
    const char *localAddress = par("localAddress");
    socket.bind(*localAddress ? L3AddressResolver().resolve(localAddress) : L3Address(), localPort);
}


void NetworkServerApp::handleMessage(cMessage *msg)
{
    if (msg->arrivedOn("socketIn")) {
        auto pkt = check_and_cast<Packet *>(msg);
        const auto &frame  = pkt->peekAtFront<LoRaMacFrame>();
        if (frame == nullptr)
            throw cRuntimeError("Header error type");
        //LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(msg);
        ///////////////////////////////////////////////////////////////////////////////////
        if(adrMethod == "LP-MAB"){
            if( simTime() >= (LP-MAB_first_stage*24*60*60)){
                totalReceivedPackets++;
            }
        }
        ///////////////////////////////////////////////////////////////////////////////////

        else
            totalReceivedPackets++;
        updateKnownNodes(pkt);
        processLoraMACPacket(pkt);
    }
    else if(msg->isSelfMessage()) {
        processScheduledPacket(msg);
    }
}

void NetworkServerApp::processLoraMACPacket(Packet *pk)
{
    const auto & frame = pk->peekAtFront<LoRaMacFrame>();
    if(isPacketProcessed(frame))
    {

        delete pk;
        return;
    }
    addPktToProcessingTable(pk);
}

void NetworkServerApp::finish()
{
    for(uint i=0;i<knownNodes.size();i++)
    {
        delete knownNodes[i].historyAllSNIR;
        delete knownNodes[i].historyAllRSSI;
        delete knownNodes[i].receivedSeqNumber;
        delete knownNodes[i].calculatedSNRmargin;
        recordScalar("Send ADR for node", knownNodes[i].numberOfSentADRPackets);
    }
    for (std::map<int,int>::iterator it=numReceivedPerNode.begin(); it != numReceivedPerNode.end(); ++it)
    {
        const std::string stringScalar = "numReceivedFromNode " + std::to_string(it->first);
        recordScalar(stringScalar.c_str(), it->second);
    }

    receivedRSSI.recordAs("receivedRSSI");
    recordScalar("totalReceivedPackets", totalReceivedPackets);

    while(!receivedPackets.empty()) {
        receivedPackets.back().endOfWaiting->removeControlInfo();
        delete receivedPackets.back().rcvdPacket;
        if (receivedPackets.back().endOfWaiting && receivedPackets.back().endOfWaiting->isScheduled()) {
            cancelAndDelete(receivedPackets.back().endOfWaiting);
        }
        else
            delete receivedPackets.back().endOfWaiting;
        receivedPackets.pop_back();
    }

    knownNodes.clear();
    receivedPackets.clear();

    recordScalar("counterUniqueReceivedPacketsPerSF SF7", counterUniqueReceivedPacketsPerSF[0]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF8", counterUniqueReceivedPacketsPerSF[1]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF9", counterUniqueReceivedPacketsPerSF[2]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF10", counterUniqueReceivedPacketsPerSF[3]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF11", counterUniqueReceivedPacketsPerSF[4]);
    recordScalar("counterUniqueReceivedPacketsPerSF SF12", counterUniqueReceivedPacketsPerSF[5]);
    if (counterOfSentPacketsFromNodesPerSF[0] > 0)
        recordScalar("DER SF7", double(counterUniqueReceivedPacketsPerSF[0]) / counterOfSentPacketsFromNodesPerSF[0]);
    else
        recordScalar("DER SF7", 0);

    if (counterOfSentPacketsFromNodesPerSF[1] > 0)
        recordScalar("DER SF8", double(counterUniqueReceivedPacketsPerSF[1]) / counterOfSentPacketsFromNodesPerSF[1]);
    else
        recordScalar("DER SF8", 0);

    if (counterOfSentPacketsFromNodesPerSF[2] > 0)
        recordScalar("DER SF9", double(counterUniqueReceivedPacketsPerSF[2]) / counterOfSentPacketsFromNodesPerSF[2]);
    else
        recordScalar("DER SF9", 0);

    if (counterOfSentPacketsFromNodesPerSF[3] > 0)
        recordScalar("DER SF10", double(counterUniqueReceivedPacketsPerSF[3]) / counterOfSentPacketsFromNodesPerSF[3]);
    else
        recordScalar("DER SF10", 0);

    if (counterOfSentPacketsFromNodesPerSF[4] > 0)
        recordScalar("DER SF11", double(counterUniqueReceivedPacketsPerSF[4]) / counterOfSentPacketsFromNodesPerSF[4]);
    else
        recordScalar("DER SF11", 0);

    if (counterOfSentPacketsFromNodesPerSF[5] > 0)
        recordScalar("DER SF12", double(counterUniqueReceivedPacketsPerSF[5]) / counterOfSentPacketsFromNodesPerSF[5]);
    else
        recordScalar("DER SF12", 0);

}

bool NetworkServerApp::isPacketProcessed(const Ptr<const LoRaMacFrame> &pkt)
{
    for(const auto & elem : knownNodes) {
        if(elem.srcAddr == pkt->getTransmitterAddress()) {
            if(elem.lastSeqNoProcessed > pkt->getSequenceNumber()) return true;
        }

    }
    return false;
}
void NetworkServerApp::updateKnownNodes(Packet* pkt)
{

    const auto & frame = pkt->peekAtFront<LoRaMacFrame>();
    bool nodeExist = false;
    for(auto &elem : knownNodes)
    {
        if(elem.srcAddr == frame->getTransmitterAddress()) {
            nodeExist = true;
            if(elem.lastSeqNoProcessed < frame->getSequenceNumber()) {
                elem.lastSeqNoProcessed = frame->getSequenceNumber();
            }
            break;
        }
    }

    if(nodeExist == false)
    {
        ////////////////////////serati//////////////////////////
        if(simTime() > 3*24*60*60)
            totalReceivedUniquePackets++;
        ////////////////////////////////////////////////////////
        knownNode newNode;
        newNode.srcAddr= frame->getTransmitterAddress();
        newNode.lastSeqNoProcessed = frame->getSequenceNumber();
        newNode.framesFromLastADRCommand = 0;
        newNode.framesFromLastStateChange = 0;
        newNode.numberOfSentADRPackets = 0;
        newNode.historyAllSNIR = new cOutVector;
        newNode.historyAllSNIR->setName("Vector of SNIR per node");
        newNode.prevSeq = 0;
        for(int i = 0; i < 10; i ++)
            for(int x = 0;x < 6; x++){
                newNode.valueFunction[i][x]= 30;
            }
        for(int i = 0; i < 10; i ++)
            for(int x = 0;x < 5; x++){
                newNode.valuePower[i][x]= 30;
            }
        for(int x = 0;x < 10; x++){
            newNode.numStateSeen[x]= 0;
        }
        newNode.prevSF =frame->getLoRaSF();
        newNode.prevState = getIndex(frame->getRSSI());
        newNode.prevTP =  math::mW2dBmW(frame->getLoRaTP()) + 30;
        //newNode.historyAllSNIR->record(pkt->getSNIR());
        newNode.historyAllSNIR->record(math::fraction2dB(frame->getSNIR()));
        newNode.historyAllRSSI = new cOutVector;
        newNode.historyAllRSSI->setName("Vector of RSSI per node");
        newNode.historyAllRSSI->record(frame->getRSSI());
        newNode.receivedSeqNumber = new cOutVector;
        newNode.receivedSeqNumber->setName("Received Sequence number");
        newNode.calculatedSNRmargin = new cOutVector;
        newNode.calculatedSNRmargin->setName("Calculated SNRmargin in ADR");
        ////////////////LP-MAB//////////////////////
        if(adrMethod == "LP-MAB"){
            newNode.c_table.end_node_sending_interval = par("sending_interval");
            newNode.c_table.init_table("LP-MAB");
            LP-MAB_first_stage = newNode.c_table.max_explr_days;

        }
        else if (adrMethod == "ADR-Lite"){
            newNode.c_table.init_table("ADR-Lite");
            newNode.c_table.tkn.addr = frame->getTransmitterAddress();
            newNode.c_table.tkn.ceil = newNode.c_table.RSSI_table_size-1;
            newNode.c_table.tkn.floor = 0;
            newNode.c_table.tkn.rtc.idx = newNode.c_table.RSSI_table_size-1;
        }
        /////////////////////////////////////
        knownNodes.push_back(newNode);
    }
}

void NetworkServerApp::addPktToProcessingTable(Packet* pkt)
{
    const auto & frame = pkt->peekAtFront<LoRaMacFrame>();
    bool packetExists = false;
    for(auto &elem : receivedPackets)
    {

        const auto &frameAux = elem.rcvdPacket->peekAtFront<LoRaMacFrame>();
        if(frameAux->getTransmitterAddress() == frame->getTransmitterAddress() && frameAux->getSequenceNumber() == frame->getSequenceNumber())
        {

            packetExists = true;
            const auto& networkHeader = getNetworkProtocolHeader(pkt);
            const L3Address& gwAddress = networkHeader->getSourceAddress();
            elem.possibleGateways.emplace_back(gwAddress, math::fraction2dB(frame->getSNIR()), frame->getRSSI());
            delete pkt;
            break;
        }
    }
    if(packetExists == false)
    {
        receivedPacket rcvPkt;
        rcvPkt.rcvdPacket = pkt;
        rcvPkt.endOfWaiting = new cMessage("endOfWaitingWindow");
        rcvPkt.endOfWaiting->setControlInfo(pkt);
        const auto& networkHeader = getNetworkProtocolHeader(pkt);
        const L3Address& gwAddress = networkHeader->getSourceAddress();
        rcvPkt.possibleGateways.emplace_back(gwAddress, math::fraction2dB(frame->getSNIR()), frame->getRSSI());
        EV << "Added " << gwAddress << " " << math::fraction2dB(frame->getSNIR()) << " " << frame->getRSSI() << endl;
        scheduleAt(simTime() + 1.2, rcvPkt.endOfWaiting);
        receivedPackets.push_back(rcvPkt);


    }
}

void NetworkServerApp::processScheduledPacket(cMessage* selfMsg)
{
    auto pkt = check_and_cast<Packet *>(selfMsg->removeControlInfo());
    const auto & frame = pkt->peekAtFront<LoRaMacFrame>();


    counterUniqueReceivedPacketsPerSF[frame->getLoRaSF()-7]++;

    L3Address pickedGateway;
    double SNIRinGW = -99999999999;
    double RSSIinGW = -99999999999;
    int packetNumber;
    int nodeNumber;
    for(uint i=0;i<receivedPackets.size();i++)
    {
        const auto &frameAux = receivedPackets[i].rcvdPacket->peekAtFront<LoRaMacFrame>();
        if(frameAux->getTransmitterAddress() == frame->getTransmitterAddress() && frameAux->getSequenceNumber() == frame->getSequenceNumber())        {
            packetNumber = i;
            nodeNumber = frame->getTransmitterAddress().getInt();
            if (numReceivedPerNode.count(nodeNumber-1)>0)
            {
                ++numReceivedPerNode[nodeNumber-1];
            } else {
                numReceivedPerNode[nodeNumber-1] = 1;
            }

            for(uint j=0;j<receivedPackets[i].possibleGateways.size();j++)
            {
                if(SNIRinGW < std::get<1>(receivedPackets[i].possibleGateways[j]))
                {
                    RSSIinGW = std::get<2>(receivedPackets[i].possibleGateways[j]);
                    SNIRinGW = std::get<1>(receivedPackets[i].possibleGateways[j]);
                    pickedGateway = std::get<0>(receivedPackets[i].possibleGateways[j]);
                }
            }
        }
    }
    emit(LoRa_ServerPacketReceived, true);
    counterUniqueReceivedPackets++;

    receivedRSSI.collect(frame->getRSSI());
    if(evaluateADRinServer)
    {
        evaluateADR(pkt, pickedGateway, SNIRinGW, RSSIinGW);
    }
    delete receivedPackets[packetNumber].rcvdPacket;
    delete selfMsg;
    receivedPackets.erase(receivedPackets.begin()+packetNumber);
}

void NetworkServerApp::evaluateADR(Packet* pkt, L3Address pickedGateway, double SNIRinGW, double RSSIinGW)
{
    bool sendADR = false;
    bool sendADRAckRep = false;
    double SNRm;
    double variance;//needed for ADR
    int nodeIndex;
    int stateIndex, sfIndex, tpIndex;
    int currentPDR;
    double avgRSSI;

    pkt->trimFront();
    auto frame = pkt->removeAtFront<LoRaMacFrame>();

    double calculatedPowerdBm = math::mW2dBmW(frame->getLoRaTP()) + 30;

    const auto & rcvAppPacket = pkt->peekAtFront<LoRaAppPacket>();

    if(rcvAppPacket->getOptions().getADRACKReq())
    {
        sendADRAckRep = true;
    }

    for(uint i=0;i<knownNodes.size();i++)
    {
        if(knownNodes[i].srcAddr == frame->getTransmitterAddress())
        {
            nodeIndex = i;
            knownNodes[i].historyAllSNIR->record(SNIRinGW);
            knownNodes[i].historyAllRSSI->record(RSSIinGW);
            knownNodes[i].receivedSeqNumber->record(frame->getSequenceNumber());

            if(knownNodes[i].adrListSNIR.size() == 20) knownNodes[i].adrListSNIR.pop_front();
            if(knownNodes[i].adrListRSSI.size() == 20) knownNodes[i].adrListRSSI.pop_front();
//            if(knownNodes[i].adrListSeqNo.size() == 20) knownNodes[i].adrListSeqNo.pop_front();
            ///////////////////Zargari///////////////////
            if(knownNodes[i].adrListSeqNo.size() == 20)  knownNodes[i].adrListSeqNo.erase(knownNodes[i].adrListSeqNo.begin());
            ////////////////////////////////////////////////////
            knownNodes[i].adrListSNIR.push_back(SNIRinGW);
            knownNodes[i].adrListRSSI.push_back(RSSIinGW);
            knownNodes[i].adrListSeqNo.push_back(frame->getSequenceNumber());
            knownNodes[i].framesFromLastADRCommand++;

            ////////////////////////////////////////////////////////////////////////////
            int wait_time_to_ack = 20;
            ////////////////////////////////////////////////////////////////////////////
            else if(adrMethod == "LP-MAB"){
                int next_setup = 0;
                int rcvd_idx;
                int rcvd_sf = frame->getLoRaSF();
                int rcvd_tp = math::mW2dBmW(frame->getLoRaTP()) + 30;
                int rcvd_cr = frame->getLoRaCR();
                units::values::Hz rcvd_bw = frame->getLoRaBW();
                units::values::Hz rcvd_cf = frame->getLoRaCF();


                rcvd_idx = knownNodes[nodeIndex].c_table.get_rssi_table_cell_index(rcvd_sf, rcvd_tp,
                        knownNodes[nodeIndex].c_table.Hz_to_int(rcvd_bw, ' '), rcvd_cr,
                        knownNodes[nodeIndex].c_table.Hz_to_int(rcvd_cf, ' '));
                knownNodes[nodeIndex].c_table.RSSI_table.at(rcvd_idx).num_server_rcvd++;
                if(knownNodes[nodeIndex].c_table.is_explr){
                    wait_time_to_ack = 9999;
                    sendADRAckRep = false;

                    if(!knownNodes[nodeIndex].c_table.explr_started){
                        knownNodes[nodeIndex].c_table.RSSI_table.at(0).num_node_sent = 1;
                        knownNodes[nodeIndex].c_table.explr_started = true;
                    }

                    if(rcvd_idx-knownNodes[nodeIndex].c_table.prev_sent_idx > 0){
                        for(int i=0, j=knownNodes[nodeIndex].c_table.prev_sent_idx+1;
                                i<rcvd_idx-knownNodes[nodeIndex].c_table.prev_sent_idx; i++,j++)
                            knownNodes[nodeIndex].c_table.RSSI_table.at(j).num_node_sent++;
                    }
                    else if(rcvd_idx-knownNodes[nodeIndex].c_table.prev_sent_idx < 0){
                        for(int i=0, j=knownNodes[nodeIndex].c_table.prev_sent_idx+1;
                                i<knownNodes[nodeIndex].c_table.RSSI_table_size - 1 - knownNodes[nodeIndex].c_table.prev_sent_idx; i++,j++)
                            knownNodes[nodeIndex].c_table.RSSI_table.at(j).num_node_sent++;
                        for(int i=0, j=0;
                                i<=rcvd_idx; i++,j++)
                            knownNodes[nodeIndex].c_table.RSSI_table.at(j).num_node_sent++;
                    }
                    else if(rcvd_idx-knownNodes[nodeIndex].c_table.prev_sent_idx == 0){
                        for(int i=0; i<=rcvd_idx; i++)
                            knownNodes[nodeIndex].c_table.RSSI_table.at(i).num_node_sent++;
                        for(int i=rcvd_idx+1; i<knownNodes[nodeIndex].c_table.RSSI_table_size; i++)
                            knownNodes[nodeIndex].c_table.RSSI_table.at(i).num_node_sent++;
                    }
                    int count = 0;
                    for(int i=0; i<knownNodes[nodeIndex].c_table.RSSI_table_size; i++){
                        if(knownNodes[nodeIndex].c_table.RSSI_table.at(i).num_node_sent >
                            knownNodes[nodeIndex].c_table.explr_itr_lim)
                            knownNodes[nodeIndex].c_table.RSSI_table.at(i).num_node_sent =
                                    knownNodes[nodeIndex].c_table.explr_itr_lim;
                        count += knownNodes[nodeIndex].c_table.RSSI_table.at(i).num_node_sent;
                    }

                    knownNodes[nodeIndex].c_table.cal_prob();
                    if(frame->getTransmitterAddress() == add){
                        knownNodes[nodeIndex].c_table.cal_weight(rcvd_idx, rcvd_idx, true);
                    }
                    else{
                        knownNodes[nodeIndex].c_table.cal_weight(rcvd_idx, rcvd_idx, false);
                    }
                    int lim = knownNodes[nodeIndex].c_table.RSSI_table_size*knownNodes[nodeIndex].c_table.explr_itr_lim;

                    if(count >= lim){
                        knownNodes[nodeIndex].c_table.exploraton_done();
                        knownNodes[i].framesFromLastADRCommand = 0;
                        sendADRAckRep = true;
                    }
                    knownNodes[nodeIndex].c_table.prev_sent_idx = rcvd_idx;
                }
                else if (knownNodes[nodeIndex].c_table.is_explt){
                    wait_time_to_ack = 20;
                    if(!knownNodes[nodeIndex].c_table.more_than_one_exploration){
                        for (int i=0; i<knownNodes[nodeIndex].c_table.RSSI_table_size; i++){
                            if(knownNodes[nodeIndex].c_table.RSSI_table.at(i).num_server_rcvd >
                                knownNodes[nodeIndex].c_table.alpha * knownNodes[nodeIndex].c_table.explotation_itr){
                                for (int i=0; i<knownNodes[nodeIndex].c_table.RSSI_table_size; i++){
                                    knownNodes[nodeIndex].c_table.RSSI_table.at(i).num_node_sent = 0;
                                }
                                knownNodes[nodeIndex].c_table.alpha ++;
                                knownNodes[nodeIndex].c_table.more_than_one_exploration = true;
                                int max = -1;
                                for (int k=0; k<knownNodes[nodeIndex].c_table.RSSI_table_size; k++){
                                    if(knownNodes[nodeIndex].c_table.RSSI_table.at(k).num_server_rcvd > max )
                                        max = knownNodes[nodeIndex].c_table.RSSI_table.at(k).num_server_rcvd;
                                }
                                for (int i=0; i<knownNodes[nodeIndex].c_table.RSSI_table_size; i++){
                                    knownNodes[nodeIndex].c_table.RSSI_table_at_least_one_recvd_array[i][0] = false;
                                    for(int j=1; j<knownNodes[nodeIndex].c_table.explr_itr_lim+1; j++)
                                        knownNodes[nodeIndex].c_table.RSSI_table_at_least_one_recvd_array[i][j] = false;
                                    if(knownNodes[nodeIndex].c_table.RSSI_table.at(i).num_server_rcvd >= max/2){
                                        knownNodes[nodeIndex].c_table.RSSI_table_at_least_one_recvd_array[i][0]= true;
                                        knownNodes[nodeIndex].c_table.RSSI_table.at(i).num_server_rcvd = 1;
                                        knownNodes[nodeIndex].c_table.RSSI_table.at(i).num_server_sent = 1;
                                    }
                                }
                                sendADRAckRep = true;
                                break;
                            }
                        }
                    }
                    else if(knownNodes[nodeIndex].c_table.more_than_one_exploration){
                        knownNodes[nodeIndex].c_table.RSSI_table.at(rcvd_idx).num_node_sent++;
                        sendADRAckRep = true;
                    }
                    knownNodes[nodeIndex].c_table.cal_prob();
                    if(frame->getTransmitterAddress() == add){
                        knownNodes[nodeIndex].c_table.cal_weight(rcvd_idx,
                                knownNodes[nodeIndex].c_table.prev_sent_idx, true);
                    }
                    else{
                        knownNodes[nodeIndex].c_table.cal_weight(rcvd_idx,
                                knownNodes[nodeIndex].c_table.prev_sent_idx, false);
                    }

                    knownNodes[nodeIndex].c_table.rcvd_idx = rcvd_idx;

                }
            }
            else if(adrMethod == "ADR-Lite" ){
                int rcvd_idx;
                int rcvd_sf = frame->getLoRaSF();
                int rcvd_tp = math::mW2dBmW(frame->getLoRaTP()) + 30;
                int rcvd_cr = frame->getLoRaCR();
                units::values::Hz rcvd_bw = frame->getLoRaBW();
                units::values::Hz rcvd_cf = frame->getLoRaCF();
                rcvd_idx = knownNodes[nodeIndex].c_table.get_rssi_table_cell_index(rcvd_sf, rcvd_tp,
                        knownNodes[nodeIndex].c_table.Hz_to_int(rcvd_bw, ' '), rcvd_cr,
                        knownNodes[nodeIndex].c_table.Hz_to_int(rcvd_cf, ' '));

            }
            if(knownNodes[i].framesFromLastADRCommand == wait_time_to_ack || sendADRAckRep){
                sendADR = true;
                if(adrMethod == "max"){
                    SNRm = *max_element(knownNodes[i].adrListSNIR.begin(), knownNodes[i].adrListSNIR.end());

                }
                else if(adrMethod == "min"){
                    SNRm = *min_element(knownNodes[i].adrListSNIR.begin(), knownNodes[i].adrListSNIR.end());
                }
                else if(adrMethod == "avg"){
                    //                    std::cout<<"AVG";
                    double totalSNR = 0;
                    int numberOfFields = 0;
                    for (std::list<double>::iterator it=knownNodes[i].adrListSNIR.begin(); it != knownNodes[i].adrListSNIR.end(); ++it)
                    {
                        totalSNR+=*it;
                        numberOfFields++;
                    }
                    SNRm = totalSNR/numberOfFields;
                }
                else if(adrMethod == "owa"){
                    double begin=knownNodes[i].adrListSeqNo.front();
                    double end=knownNodes[i].adrListSeqNo.back();
                    double size=knownNodes[i].adrListSeqNo.size();
                    double pathloss=(end-begin-18)/(end-begin);
                    double pathlossRate = (int)(pathloss * 100000 + .5);
                    pathlossRate=pathlossRate / 100000;
                    knownNodes[i].adrListSNIR.sort();
                    double totalSNR = 0;
                    double result=0;
                    int last = 19;
                    for (std::list<double>::iterator j=knownNodes[i].adrListSNIR.begin(); j != knownNodes[i].adrListSNIR.end(); ++j)
                    {                   //pessimistic owa ADR
                        if(last==1){
                            result=pow(pathlossRate,(19-last));
                        }else{
                            result=(1-pathlossRate)*pow(pathlossRate,(19-last));
                        }
                        result = (int)(result * 100000 + .5);
                        result=result / 100000;
                        totalSNR=(*j * result)+totalSNR;
                        last=last-1;
                    }
                    SNRm = totalSNR;
                }
                //  else
                knownNodes[i].framesFromLastADRCommand = 0;
            }
            break;
        }
    }

    if(sendADR )
    {
        LoRaOptions newOptions;
        int calculatedSF = frame->getLoRaSF();
        double calculatedPowerdBm = math::mW2dBmW(frame->getLoRaTP()) + 30;// math::mW2dBmW(frame->getLoRaTP()) ;
        ////////////////////////////////////////////////////////////
        int calculatedCR = frame->getLoRaCR();
        units::values::Hz calculatedBWHz = frame->getLoRaBW();
        units::values::Hz calculatedCFHz = frame->getLoRaCF();
        /////////////////////////////////////////////////////////
        auto mgmtPacket = makeShared<LoRaAppPacket>();
        mgmtPacket->setMsgType(TXCONFIG);
        if(adrMethod == "ADR-Lite"){
            int rcvd_idx;
            int rcvd_sf = frame->getLoRaSF();
            int rcvd_tp = math::mW2dBmW(frame->getLoRaTP()) + 30;
            int rcvd_cr = frame->getLoRaCR();
            units::values::Hz rcvd_bw = frame->getLoRaBW();
            units::values::Hz rcvd_cf = frame->getLoRaCF();

            rcvd_idx = knownNodes[nodeIndex].c_table.get_rssi_table_cell_index(rcvd_sf, rcvd_tp,
                                    knownNodes[nodeIndex].c_table.Hz_to_int(rcvd_bw, ' '), rcvd_cr,
                                    knownNodes[nodeIndex].c_table.Hz_to_int(rcvd_cf, ' '));
            int next_setup = knownNodes[nodeIndex].c_table.RSSI_table_size-1;
            if(knownNodes[nodeIndex].c_table.tkn.rtc.idx == rcvd_idx){//Everything sounds great
                knownNodes[nodeIndex].c_table.tkn.floor = 0;
                knownNodes[nodeIndex].c_table.tkn.ceil = rcvd_idx;
            }
            else if (rcvd_idx < knownNodes[nodeIndex].c_table.tkn.rtc.idx){//There is great chance that node hadn't received Ack

            }
            else if (rcvd_idx > knownNodes[nodeIndex].c_table.tkn.rtc.idx){//Auto recovery process at node side enabled
                knownNodes[nodeIndex].c_table.tkn.floor = knownNodes[nodeIndex].c_table.tkn.rtc.idx;
                knownNodes[nodeIndex].c_table.tkn.ceil =  rcvd_idx;
            }

            next_setup = floor((knownNodes[nodeIndex].c_table.tkn.floor + knownNodes[nodeIndex].c_table.tkn.ceil) / 2);
            knownNodes[nodeIndex].c_table.tkn.rtc = knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup);
            knownNodes[nodeIndex].c_table.tkn.rtc.idx = next_setup;

            calculatedSF = knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).sf;
            calculatedPowerdBm = knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).tp;
            calculatedBWHz = knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).loRaBW;
            calculatedCFHz= knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).loRaCF;
            calculatedCR = knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).cr;
        }
        else if(adrMethod == "LP-MAB"){
            int next_setup=-1;
            if(knownNodes[nodeIndex].c_table.more_than_one_exploration){
                for (int j=1; j<knownNodes[nodeIndex].c_table.explr_itr_lim+1&&next_setup==-1; j++){
                    for (int i=0; i<knownNodes[nodeIndex].c_table.RSSI_table_size&&next_setup==-1; i++){
                        if (knownNodes[nodeIndex].c_table.RSSI_table_at_least_one_recvd_array[i][0] == true){
                            if(knownNodes[nodeIndex].c_table.RSSI_table_at_least_one_recvd_array[i][j] == false){
                                knownNodes[nodeIndex].c_table.RSSI_table_at_least_one_recvd_array[i][j] = true;
                                knownNodes[nodeIndex].c_table.RSSI_table_at_least_one_recvd_cnt++;
                                next_setup = i;
                            }
                        }
                    }
                }
                if(next_setup == -1){
                    knownNodes[nodeIndex].c_table.more_than_one_exploration = false;
                }
            }
            if(!knownNodes[nodeIndex].c_table.more_than_one_exploration){
                next_setup = knownNodes[nodeIndex].c_table.select_nxt_idx("LP-MAB", "");
            }
            knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).num_server_sent++;
            knownNodes[nodeIndex].c_table.prev_sent_idx = next_setup;
            calculatedSF = knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).sf;
            calculatedPowerdBm = knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).tp;
            calculatedBWHz = knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).loRaBW;
            calculatedCFHz= knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).loRaCF;
            calculatedCR = knownNodes[nodeIndex].c_table.RSSI_table.at(next_setup).cr;
        }
        else{
            double SNRmargin;
            double requiredSNR;
            if(frame->getLoRaSF() == 7) requiredSNR = -7.5;
            if(frame->getLoRaSF() == 8) requiredSNR = -10;
            if(frame->getLoRaSF() == 9) requiredSNR = -12.5;
            if(frame->getLoRaSF() == 10) requiredSNR = -15;
            if(frame->getLoRaSF() == 11) requiredSNR = -17.5;
            if(frame->getLoRaSF() == 12) requiredSNR = -20;

            SNRmargin = SNRm - requiredSNR - adrDeviceMargin;
            knownNodes[nodeIndex].calculatedSNRmargin->record(SNRmargin);
            int Nstep = round(SNRmargin/3);

            // Increase the data rate with each step

            while(Nstep > 0 && calculatedSF > 7)
            {
                calculatedSF--;
                Nstep--;
            }

            // Decrease the Tx power by 3 for each step, until min reached

            while(Nstep > 0 && calculatedPowerdBm > 2)
            {
                calculatedPowerdBm-=3;
                Nstep--;
            }
            if(calculatedPowerdBm < 2) calculatedPowerdBm = 2;

            // Increase the Tx power by 3 for each step, until max reached
            while(Nstep < 0 && calculatedPowerdBm < 14){
                calculatedPowerdBm+=3;
                Nstep++;
            }
            if(calculatedPowerdBm > 14)
                calculatedPowerdBm = 14;
        }

        newOptions.setLoRaSF(calculatedSF);
        newOptions.setLoRaTP(calculatedPowerdBm);
        if(adrMethod == "LP-MAB" || adrMethod == "ADR-Lite"){
            newOptions.setLoRaBW(knownNodes[nodeIndex].c_table.Hz_to_int(calculatedBWHz, 'k'));
            newOptions.setLoRaCF(knownNodes[nodeIndex].c_table.Hz_to_int(calculatedCFHz, 'm'));
            newOptions.setLoRaCR(calculatedCR);
        }
        EV << calculatedSF << endl;
        EV << calculatedPowerdBm << endl;
        mgmtPacket->setOptions(newOptions);

        else if( sendADR == true)
            knownNodes[nodeIndex].numberOfSentADRPackets++;

        auto frameToSend = makeShared<LoRaMacFrame>();
        frameToSend->setChunkLength(B(par("headerLength").intValue()));

        //  LoRaMacFrame *frameToSend = new LoRaMacFrame("ADRPacket");

        //frameToSend->encapsulate(mgmtPacket);
        frameToSend->setReceiverAddress(frame->getTransmitterAddress());
        //FIXME: What value to set for LoRa TP
        //        frameToSend->setLoRaTP(pkt->getLoRaTP());
        frameToSend->setLoRaTP(math::dBmW2mW(14));
        frameToSend->setLoRaCF(frame->getLoRaCF());
        frameToSend->setLoRaSF(frame->getLoRaSF());
        frameToSend->setLoRaBW(frame->getLoRaBW());

        auto pktAux = new Packet("ADRPacket");
        mgmtPacket->setChunkLength(B(par("headerLength").intValue()));

        pktAux->insertAtFront(mgmtPacket);
        pktAux->insertAtFront(frameToSend);
        socket.sendTo(pktAux, pickedGateway, destPort);
    }
    //delete pkt;
}

void NetworkServerApp::receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details)
{
    if(adrMethod == "LP-MAB"){
        if( simTime() >= (LP-MAB_first_stage*24*60*60)){
            counterOfSentPacketsFromNodes++;
            counterOfSentPacketsFromNodesPerSF[value-7]++;
        }
    }
    else{
        counterOfSentPacketsFromNodes++;
        counterOfSentPacketsFromNodesPerSF[value-7]++;
    }
}

} //namespace inet
