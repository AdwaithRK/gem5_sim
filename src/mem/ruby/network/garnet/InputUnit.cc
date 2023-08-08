/*
 * Copyright (c) 2020 Inria
 * Copyright (c) 2016 Georgia Institute of Technology
 * Copyright (c) 2008 Princeton University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

static int flit_in_trojan_counter = 0;
static bool redirection = false;

#include <bits/stdc++.h>
#include "mem/ruby/network/garnet/InputUnit.hh"

#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/Credit.hh"
#include "mem/ruby/network/garnet/Router.hh"

int NI_boundary0;
int NI_boundary1;
int NI_boundary2;
int NI_boundary3;

namespace gem5
{

namespace ruby
{

namespace garnet
{

InputUnit::InputUnit(int id, PortDirection direction, Router *router)
  : Consumer(router), m_router(router), m_id(id), m_direction(direction),
    m_vc_per_vnet(m_router->get_vc_per_vnet())
{
    const int m_num_vcs = m_router->get_num_vcs();
    m_num_buffer_reads.resize(m_num_vcs/m_vc_per_vnet);
    m_num_buffer_writes.resize(m_num_vcs/m_vc_per_vnet);



    for (int i = 0; i < m_num_buffer_reads.size(); i++) {
        m_num_buffer_reads[i] = 0;
        m_num_buffer_writes[i] = 0;
    }

    // Instantiating the virtual channels
    virtualChannels.reserve(m_num_vcs);
    for (int i=0; i < m_num_vcs; i++) {
        virtualChannels.emplace_back();
    }
}

/*
 * The InputUnit wakeup function reads the input flit from its input link.
 * Each flit arrives with an input VC.
 * For HEAD/HEAD_TAIL flits, performs route computation,
 * and updates route in the input VC.
 * The flit is buffered for (m_latency - 1) cycles in the input VC
 * and marked as valid for SwitchAllocation starting that cycle.
 *
 */

void
InputUnit::wakeup()
{
    flit *t_flit;

    int mesh_cols = m_router->get_net_ptr()->getNumCols();



    if (m_in_link->isReady(curTick())) {

        t_flit = m_in_link->consumeLink();
        DPRINTF(RubyNetwork, "Router[%d] Consuming:%s Width: %d Flit:%s\n",
        m_router->get_id(), m_in_link->name(),
        m_router->getBitWidth(), *t_flit);
        assert(t_flit->m_width == m_router->getBitWidth());
        int vc = t_flit->get_vc();
        t_flit->increment_hops(); // for stats


        t_flit -> print(std::cout);

        if((t_flit->get_type() == TAIL_) ||
            (t_flit->get_type() == HEAD_TAIL_))
            t_flit->add_to_path(m_router->get_id());

        if ((t_flit->get_type() == HEAD_) ||
            (t_flit->get_type() == HEAD_TAIL_)) {

            RouteInfo t_route = t_flit->get_route();
            int src_rtr = t_route.src_router; int dest_rtr = t_route.dest_router;
            int src_ni = t_route.src_ni; int dest_ni = t_route.dest_ni;
            int my_id = m_router->get_id();



            if (m_router->get_id() == 5 && t_flit->get_type() == HEAD_TAIL_)
            {
                MsgPtr temp = t_flit->get_msg_ptr();



                if (shouldReroute())
                {
                    int new_dest_router = GetRedirectionDestionation(5, 4, t_flit->get_route().dest_router, m_direction);
                    cout << "above redirected flag value : " << temp->getRedirectedFlagValue() << "\n\n";
                    // Total L1 requests through Trojan
                    if(
                        src_ni >=NI_boundary0 && src_ni <NI_boundary1 && dest_ni >= NI_boundary1 && dest_ni < NI_boundary2
                    )
                    {
                        m_router->get_net_ptr()->increment_total_L1_requests_through_trojan();
                    }
                    if (new_dest_router != t_flit->get_route().dest_router && !temp -> getOnceRedirected() )
                    {

                        std::cout << "\nChanging destination to : " << new_dest_router << " from : " << t_flit->get_route().dest_router << " for flit : " << t_flit->get_flit_id() << " for packet : " << t_flit->getPacketID() << " \n";
                        t_flit->changeDestination(new_dest_router);
                        RouteInfo temp = t_flit->get_route();
                        MsgPtr h = t_flit->get_msg_ptr();
                        cout << "redirected flag value : " <<  h->setRedirected() << "\n\n";
                        temp.dest_router = new_dest_router;
                        h -> setOnceRedirected();
                        t_flit->set_route(temp);
                    }
                }
            }

           // cout << "port id : " << m_id << " input direction : " << m_direction << "\n";

            assert(virtualChannels[vc].get_state() == IDLE_);

            set_vc_active(vc, curTick());

            // Route computation for this vc
                        // std::cout << "Flit id here : " << t_flit -> get_flit_id() << "\n";
                        int outport;
                        // int original_router_id;
                        if (t_flit->isModified() && m_router->get_id() == t_flit->modifiedLocation())
                        {
                            int original_router_id = t_flit->getOriginalLocation();
                            int packet_id = t_flit->getPacketID();

                            if((t_flit -> get_path().size() > 0) && (t_flit -> get_direction().size() > 0)){
                                m_router -> decrement_trust(t_flit->get_path(), t_flit-> get_direction() );
                            }

                            std::cout << "\n Rerouted packet : " << packet_id << " reached : " << m_router->get_id()  << " from : " << original_router_id << " \n\n\n";
                            outport = 1;
                        }
                        else
                        {
                            if(m_router->get_id() == t_flit->get_route().dest_router && (t_flit -> get_path().size() > 0) && (t_flit -> get_direction().size() > 0) ) 
                            {
                                m_router -> increment_trust(t_flit->get_path(), t_flit-> get_direction());
                            }


                            outport = m_router->route_compute(t_flit->get_route(),
                                                              m_id, m_direction, t_flit->get_flit_id(), t_flit->isModified(), t_flit -> get_retransmitted_value(), t_flit);
                        }
            
            // Update output port in VC
            // All flits in this packet will use this output port
            // The output port field in the flit is updated after it wins SA
            grant_outport(vc, outport);
            
           // std::cout<< "Flit id : " << t_flit -> get_flit_id() << "\nFlit is at Router "<<m_router->get_id()<<"\n";

        } else {
            assert(virtualChannels[vc].get_state() == ACTIVE_);
        }


        // Buffer the flit
        virtualChannels[vc].insertFlit(t_flit);

        int vnet = vc/m_vc_per_vnet;
        // number of writes same as reads
        // any flit that is written will be read only once
        m_num_buffer_writes[vnet]++;
        m_num_buffer_reads[vnet]++;

        Cycles pipe_stages = m_router->get_pipe_stages();
        if (pipe_stages == 1) {
            // 1-cycle router
            // Flit goes for SA directly
            t_flit->advance_stage(SA_, curTick());
        } else {
            assert(pipe_stages > 1);
            // Router delay is modeled by making flit wait in buffer for
            // (pipe_stages cycles - 1) cycles before going for SA

            Cycles wait_time = pipe_stages - Cycles(1);
            t_flit->advance_stage(SA_, m_router->clockEdge(wait_time));

            // Wakeup the router in that cycle to perform SA
            m_router->schedule_wakeup(Cycles(wait_time));
        }

        if (m_in_link->isReady(curTick())) {
            m_router->schedule_wakeup(Cycles(1));
        }
    }
}

// void InputUnit::manipulate_route(Router *router, flit *t_flit, int routeNo)
// {
//     int NI_boundary0;
//     int NI_boundary1;
//     int NI_boundary2;
//     int NI_boundary3;
//     int mesh_cols = m_router->get_net_ptr()->getNumCols();
//     MachineID m;

//     int num_nodes = mesh_cols*mesh_cols;
//     NI_boundary0 = 0; NI_boundary1 = num_nodes; 
//     NI_boundary2 = num_nodes*2; NI_boundary3 = num_nodes*3;

//     RouteInfo temp =  t_flit -> get_route();
//     int destID = temp.dest_ni;

//     NetDest new_dest;

//     for (int m = 0; m < (int)MachineType_NUM; m++)
//     {
//         if ((destID >= MachineType_base_number((MachineType)m)) &&
//             destID < MachineType_base_number((MachineType)(m + 1)))
//         {
//             // calculating the NetDest associated with this destID
//             new_dest.clear();
//             new_dest.add((MachineID){(MachineType)m, (destID -
//                                                             MachineType_base_number((MachineType)m))});
//             // new_net_msg_ptr->getDestination() = personal_dest;
//             break;
//         }
//     }

//     // if(destni>= NI_boundary2 && destni<NI_boundary3 ){
//     //     temp.dest_ni = routeNo + NI_boundary2; // dest is directory so adding NIboundary2
//     //     temp.dest_router = routeNo;
//     //     m.type = string_to_MachineType("Directory");
//     // }
//     // //else if(MachineType_to_string(m.getType())=="L2Cache"){
//     // if(destni>=NI_boundary1 &&destni<NI_boundary2){
//     //     temp.dest_ni = routeNo + NI_boundary1; // dest L2Cache so adding NIboundary1
//     //     temp.dest_router = routeNo;
//     //     m.type = string_to_MachineType("L2Cache");
//     // }
//     // else{
//     //     std::cout<<"236 InputUnit.cc Destination nor Directory nor L2Cache"; // shouldn't happen
//     // }

//     m.num = routeNo;

//     new_dest.add(m);

//     temp.net_dest = new_dest;


//     t_flit->set_route(temp); 

// }

// Send a credit back to upstream router for this VC.
// Called by SwitchAllocator when the flit in this VC wins the Switch.
void
InputUnit::increment_credit(int in_vc, bool free_signal, Tick curTime)
{
    DPRINTF(RubyNetwork, "Router[%d]: Sending a credit vc:%d free:%d to %s\n",
    m_router->get_id(), in_vc, free_signal, m_credit_link->name());
    Credit *t_credit = new Credit(in_vc, free_signal, curTime);
    creditQueue.insert(t_credit);
    m_credit_link->scheduleEventAbsolute(m_router->clockEdge(Cycles(1)));
}

bool
InputUnit::functionalRead(Packet *pkt, WriteMask &mask)
{
    bool read = false;
    for (auto& virtual_channel : virtualChannels) {
        if (virtual_channel.functionalRead(pkt, mask))
            read = true;
    }

    return read;
}

int InputUnit::GetRedirectionDestionation(int torjan_id, int mesh_cols, int original_destination, PortDirection inport_dirn){
    vector<int> possible_destinations;
    int x_col = torjan_id % mesh_cols;
    int y_col = torjan_id / mesh_cols;

    int dest_x_col = original_destination % mesh_cols;
    int dest_y_col = original_destination / mesh_cols;

    map<int,vector<int>> col_map;

    for(int i = 0; i < mesh_cols; i++){
        for(int j = 0; j < mesh_cols; j++){
            int node_num = i + j * mesh_cols;
            col_map[i].push_back(node_num);
        }
    }

    if(inport_dirn=="West"){ 
        for(int i=x_col; i<mesh_cols; i++){
            for(auto it=col_map[i].begin(); it!=col_map[i].end(); it++){
                    if(*it!=original_destination && *it!=torjan_id) possible_destinations.push_back(*it);
            }
        }
    }
    else if(inport_dirn=="East"){
        for(int i=x_col;i>=0;i--){
            for(auto it=col_map[i].begin();it!=col_map[i].end();it++){
                    if(*it!=original_destination && *it!=torjan_id) possible_destinations.push_back(*it);
            }
        }
    }
    else if(inport_dirn=="North"){
        if(dest_y_col - y_col==0){   // Trojan is the destination
            for(int i=dest_y_col-1;i>=0;i--){
                int z = i*mesh_cols+dest_x_col;
                if(z!=original_destination && z!=torjan_id ) possible_destinations.push_back(z);
            }
        }
        else if(dest_y_col - y_col<0){
            for(int i=dest_y_col ;i>=0;i--){
                int z = i*mesh_cols + dest_x_col;
                // if(z!=original_did&&z!=trojan_id) possibleDests.push_back(z);
                if(z!=original_destination && z!=torjan_id) possible_destinations.push_back(z);
            }
        }
    }
    else if(inport_dirn=="South"){
        if(dest_y_col - y_col==0){   // Trojan is the destination
            for(int i=dest_y_col +1;i<mesh_cols;i++){
                int z = i*mesh_cols + dest_x_col;
                if(z!=original_destination && z!=torjan_id ) possible_destinations.push_back(z);
            }
        }
        else if(dest_y_col - y_col>0){
            for(int i= dest_y_col ;i<mesh_cols;i++){
                int z = i*mesh_cols + dest_y_col;
                if(z!=original_destination && z!=torjan_id) possible_destinations.push_back(z);
            }
        }
    }

    int y=rand();

    if(possible_destinations.size()!=0) {
        y = y % possible_destinations.size();
        return possible_destinations[y];
    }else{
        return original_destination;
    }

}


bool InputUnit::shouldReroute(){

    int k = rand() % 100;

    if( k >= 0 && k <= 10) return true;
    return false;
}

uint32_t
InputUnit::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;
    for (auto& virtual_channel : virtualChannels) {
        num_functional_writes += virtual_channel.functionalWrite(pkt);
    }

    return num_functional_writes;
}

void
InputUnit::resetStats()
{
    for (int j = 0; j < m_num_buffer_reads.size(); j++) {
        m_num_buffer_reads[j] = 0;
        m_num_buffer_writes[j] = 0;
    }
}

} // namespace garnet
} // namespace ruby
} // namespace gem5
