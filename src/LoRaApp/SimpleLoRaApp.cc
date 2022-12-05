//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "SimpleLoRaApp.h"
#include "inet/mobility/static/StationaryMobility.h"
#include "inet/mobility/single/RandomWaypointMobility.h"
#include "../LoRa/LoRaTagInfo_m.h"
#include "inet/common/packet/Packet.h"
#include "unistd.h"

namespace flora {

Define_Module(SimpleLoRaApp);

static int my_node_address_test_count = 0;

void SimpleLoRaApp::initialize(int stage)
{
    /////////////////////////////////////////////////////
    adrMethod = par("myADR").stdstringValue();
    if(adrMethod == "LP-MAB" ){
        my_node_address_test_count ++;
        addrrss = my_node_address_test_count;
        if(my_node_address_test_count == 1){
            addrrss =  999;
        }
        if (!c_table.list_done){
            c_table.init_table("LP-MAB" );
            c_table.list_done = true;
        }
    }
    else if(adrMethod == "ADR-Lite"){
        loRaTP = 14;
        loRaSF = 12;
        loRaBW = inet::units::values::kHz(125);
        loRaCR = 4;
        if (!c_table.list_done){
            c_table.init_table("ADR-Lite");
            c_table.list_done = true;
        }
    }

    /////////////////////////////////////////////////////
    cSimpleModule::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        std::pair<double,double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);
        // Generate random location for nodes if circle deployment type
        if(par("isMobile")){
            RandomWaypointMobility *mobility = check_and_cast<RandomWaypointMobility *>(host->getSubmodule("mobility"));
                           mobility->par("speed").setDoubleValue(par("speed"));
                       }
        if (strcmp(host->par("deploymentType").stringValue(), "circle")==0) {
           coordsValues = generateUniformCircleCoordinates(host->par("maxGatewayDistance").doubleValue(), host->par("gatewayX").doubleValue(), host->par("gatewayY").doubleValue());
           StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
           mobility->par("initialX").setDoubleValue(coordsValues.first);
           mobility->par("initialY").setDoubleValue(coordsValues.second);
        }
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
        if (!isOperational)
            throw cRuntimeError("This module doesn't support starting in node DOWN state");
        do {
            timeToFirstPacket = par("timeToFirstPacket");
            EV << "Wylosowalem czas :" << timeToFirstPacket << endl;
            //if(timeToNextPacket < 5) error("Time to next packet must be grater than 3");
        } while(timeToFirstPacket <= 5);

        //timeToFirstPacket = par("timeToFirstPacket");
        sendMeasurements = new cMessage("sendMeasurements");
        scheduleAt(simTime()+timeToFirstPacket, sendMeasurements);

        sentPackets = 0;
        receivedADRCommands = 0;
        numberOfPacketsToSend = par("numberOfPacketsToSend");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        //LoRa physical layer parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaCR = par("initialLoRaCR");
        loRaUseHeader = par("initialUseHeader");

        evaluateADRinNode = par("evaluateADRinNode");
        sfVector.setName("SF Vector");
        tpVector.setName("TP Vector");
    }
}

std::pair<double,double> SimpleLoRaApp::generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY)
{
    double randomValueRadius = uniform(0,(radius*radius));
    double randomTheta = uniform(0,2*M_PI);

    // generate coordinates for circle with origin at 0,0
    double x = sqrt(randomValueRadius) * cos(randomTheta);
    double y = sqrt(randomValueRadius) * sin(randomTheta);
    // Change coordinates based on coordinate system used in OMNeT, with origin at top left
    x = x + gatewayX;
    y = gatewayY - y;
    std::pair<double,double> coordValues = std::make_pair(x,y);
    return coordValues;
}

void SimpleLoRaApp::finish()
{
    cModule *host = getContainingNode(this);
    if(par("isMobile")){
        RandomWaypointMobility *mobility = check_and_cast<RandomWaypointMobility *>(host->getSubmodule("mobility"));
            Coord coord = mobility->getCurrentPosition();
            recordScalar("positionX", coord.x);
            recordScalar("positionY", coord.y);
    }
    else{
    StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
    recordScalar("positionX", coord.x);
    recordScalar("positionY", coord.y);
}
    recordScalar("finalTP", loRaTP);
    recordScalar("finalSF", loRaSF);
    recordScalar("sentPackets", sentPackets);
    recordScalar("receivedADRCommands", receivedADRCommands);
}

void SimpleLoRaApp::handleMessage(cMessage *msg)
{

    if (msg->isSelfMessage()) {
        if (msg == sendMeasurements)
        {
            sendJoinRequest();
            if (adrMethod == "LP-MAB"){
                if(simTime() >= (c_table.max_explr_days*24*60*60)){
                    sentPackets++;
                }
            }else{
                sentPackets++;
            }
            delete msg;
            if(numberOfPacketsToSend == 0 || sentPackets < numberOfPacketsToSend)
            {
                double time;
                if(loRaSF == 7) time = 7.808;
                if(loRaSF == 8) time = 13.9776;
                if(loRaSF == 9) time = 24.6784;
                if(loRaSF == 10) time = 49.3568;
                if(loRaSF == 11) time = 85.6064;
                if(loRaSF == 12) time = 171.2128;

                if(adrMethod == "serati"){
                    //////////////////////////////////////////////////////////////////////////////////////////
                    int var = 2; //ToDo
                    //////////////////////////////////////////////////////////////////////////////////////////
//                    c_table.Interval_size = ceil(time/100) * var;
                    do {
//                        count ++;
//                        if(count > 1)
//                            cout << "BOOBya\t" << count << " : " << timeToNextPacket << "\n";
                        timeToNextPacket = par("timeToNextPacket");

                        // ToDo: check whether you can import delay/receive window time from mac-file to supersede with 4
                    } while(timeToNextPacket <= time || timeToNextPacket <= c_table.repli_max_interval * c_table.repli_interval_time + 4);

                }
                else{
                    do {
                        timeToNextPacket = par("timeToNextPacket");
                        //if(timeToNextPacket < 3) error("Time to next packet must be grater than 3");
                    } while(timeToNextPacket <= time);
                }
                sendMeasurements = new cMessage("sendMeasurements");
                scheduleAt(simTime() + timeToNextPacket, sendMeasurements);
            }
        }
    }
    else
    {

        handleMessageFromLowerLayer(msg);
        delete msg;
        //cancelAndDelete(sendMeasurements);
        //sendMeasurements = new cMessage("sendMeasurements");
        //scheduleAt(simTime(), sendMeasurements);
    }
}

void SimpleLoRaApp::handleMessageFromLowerLayer(cMessage *msg)
{
    auto pkt = check_and_cast<Packet *>(msg);
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();
    if(adrMethod == "LP-MAB"){
        sendNextPacketWithADRACKReq = false;
        if( simTime() >= (c_table.max_explr_days*24*60*60))
            receivedADRCommands++;
    }
    else{
        receivedADRCommands++;
    }
    if(evaluateADRinNode)
    {
        ADR_ACK_CNT = 0;
        if(packet->getMsgType() == TXCONFIG){
//            if(adrMethod == "serati");
//            else{
                if(packet->getOptions().getLoRaTP() != -1)
                    loRaTP = packet->getOptions().getLoRaTP();
                if(packet->getOptions().getLoRaSF() != -1){
                    loRaSF = packet->getOptions().getLoRaSF();
                }
//            }
            EV << "New TP " << loRaTP << endl;
            EV << "New SF " << loRaSF << endl;

            if(adrMethod == "LP-MAB" || adrMethod == "ADR-Lite"){
                if(packet->getOptions().getLoRaBW() != -1)
                    loRaBW = c_table.int_to_Hz(packet->getOptions().getLoRaBW(),' ');
                if(packet->getOptions().getLoRaCF() != -1)
                    loRaCF = c_table.int_to_Hz(packet->getOptions().getLoRaCF(),' ');
                if(packet->getOptions().getLoRaCR() != -1)
                    loRaCR = packet->getOptions().getLoRaCR();
            }
            if(adrMethod == "LP-MAB"){
                int rcvd_idx = c_table.get_rssi_table_cell_index(loRaSF, loRaTP, c_table.Hz_to_int(loRaBW, ' '),
                        loRaCR, c_table.Hz_to_int(loRaCF, ' '));


                if(c_table.is_explr){
                        c_table.RSSI_table.at(rcvd_idx).num_node_rcvd++;
                }
            }
            else if(adrMethod == "ADR-Lite"){
                int rcvd_idx = c_table.get_rssi_table_cell_index(loRaSF, loRaTP, c_table.Hz_to_int(loRaBW, ' '),
                                        loRaCR, c_table.Hz_to_int(loRaCF, ' '));
            }
        }
    }
}

bool SimpleLoRaApp::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

void SimpleLoRaApp::sendJoinRequest()
{
    auto pktRequest = new Packet("DataFrame");
    pktRequest->setKind(DATA);

    auto payload = makeShared<LoRaAppPacket>();
    payload->setChunkLength(B(par("dataSize").intValue()));

    lastSentMeasurement = rand();
    payload->setSampleMeasurement(lastSentMeasurement);

    if(evaluateADRinNode && sendNextPacketWithADRACKReq)
    {
        auto opt = payload->getOptions();
        opt.setADRACKReq(true);
        payload->setOptions(opt);
        //request->getOptions().setADRACKReq(true);0
        sendNextPacketWithADRACKReq = false;
    }

    if(adrMethod == "LP-MAB"){
        int prev_sent_idx = c_table.prev_sent_idx;

        if(c_table.is_explr){
            loRaBW = c_table.RSSI_table.at(prev_sent_idx).loRaBW;
            loRaCF = c_table.RSSI_table.at(prev_sent_idx).loRaCF;
            loRaSF = c_table.RSSI_table.at(prev_sent_idx).sf;
            loRaCR = c_table.RSSI_table.at(prev_sent_idx).cr;
//            loRaTP = mW(math::dBmW2mW(c_table.RSSI_table.at(prev_sent_idx).tp));
            loRaTP = c_table.RSSI_table.at(prev_sent_idx).tp;
            c_table.RSSI_table.at(prev_sent_idx).num_node_sent++;
            prev_sent_idx++;
            if(prev_sent_idx == c_table.RSSI_table_size){
                prev_sent_idx = 0;
            }
            c_table.prev_sent_idx = prev_sent_idx;
        }

    }
    else if(adrMethod == "serati"){
        sendNextPacketWithADRACKReq = true;
        int prev_sent_idx = c_table.prev_sent_idx;
        if(c_table.is_explr){
            loRaBW = c_table.RSSI_table.at(prev_sent_idx).loRaBW;
            loRaCF = c_table.RSSI_table.at(prev_sent_idx).loRaCF;
            loRaSF = c_table.RSSI_table.at(prev_sent_idx).sf;
//            loRaSF = 12;
            loRaCR = c_table.RSSI_table.at(prev_sent_idx).cr;
//            loRaTP = mW(math::dBmW2mW(c_table.RSSI_table.at(prev_sent_idx).tp));
            loRaTP = c_table.RSSI_table.at(prev_sent_idx).tp;
            c_table.RSSI_table.at(prev_sent_idx).num_node_sent++;
            c_table.repli_max_interval = c_table.RSSI_table.at(prev_sent_idx).retransmit_num;
            c_table.repli_interval_time = c_table.RSSI_table.at(prev_sent_idx).retransmit_intv;
            prev_sent_idx++;
            if(prev_sent_idx == c_table.RSSI_table_size){
                prev_sent_idx = 0;
            }
            c_table.prev_sent_idx = prev_sent_idx;
        }
    }

    auto loraTag = pktRequest->addTagIfAbsent<LoRaTag>();
    loraTag->setBandwidth(loRaBW);
    loraTag->setCenterFrequency(loRaCF);
    loraTag->setSpreadFactor(loRaSF);
    loraTag->setCodeRendundance(loRaCR);
    loraTag->setPower(mW(math::dBmW2mW(loRaTP)));

    sfVector.record(loRaSF);
    tpVector.record(loRaTP);
    pktRequest->insertAtBack(payload);
    send(pktRequest, "appOut");

    if(adrMethod=="LP-MAB"){
        c_table.node_total_pckt_sent++;
        if(c_table.node_total_pckt_sent >= c_table.explr_itr_lim * c_table.RSSI_table_size &&
                c_table.is_explr){
            ADR_ACK_CNT = 0;
            MacAddress add (0x0AAA00999999);
            c_table.exploraton_done();
        }
    }
    /////////////////////////////////////////////////////////////////////////////////////////
    if(evaluateADRinNode)
    {
        ADR_ACK_CNT++;
        if(adrMethod=="LP-MAB"){
            int t1=5;
            int t2=10;
            if(c_table.is_explt){
                if(ADR_ACK_CNT == t1){
                    sendNextPacketWithADRACKReq = true;
                }
                if(ADR_ACK_CNT >= t1 + t2){
                    ADR_ACK_CNT = 0;
                    increaseSFIfPossible();
                }
            }
        }
        else if(adrMethod == "ADR-Lite"){
            int t1=5;
            int t2=10;
            if(ADR_ACK_CNT == t1){
                sendNextPacketWithADRACKReq = true;
            }
            if(ADR_ACK_CNT >= t1 + t2){
                ADR_ACK_CNT = 0;
                increaseSFIfPossible();
                }
        }
        else{
            if(ADR_ACK_CNT == ADR_ACK_LIMIT){
                sendNextPacketWithADRACKReq = true;
            }
            if(ADR_ACK_CNT >= ADR_ACK_LIMIT + ADR_ACK_DELAY){
                ADR_ACK_CNT = 0;
                increaseSFIfPossible();
            }
        }
    }
        emit(LoRa_AppPacketSent, loRaSF);

}

void SimpleLoRaApp::increaseSFIfPossible()
{
    if (adrMethod == "ADR-Lite"){
        if(loRaSF < 12)
            loRaSF++;
        if(loRaTP < 14)
            loRaTP+=3;
    }
    else{ 
        if(loRaSF<12) {
            loRaSF ++;
    }

}

} //end namespace inet
