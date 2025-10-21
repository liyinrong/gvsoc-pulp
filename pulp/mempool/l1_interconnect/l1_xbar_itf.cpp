/*
 * Copyright (C) 2025 ETH Zurich and University of Bologna
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Authors: Yinrong Li, ETH Zurich (yinrli@student.ethz.ch)
 */

#include <vp/vp.hpp>
#include <vp/signal.hpp>
#include <vp/itf/io.hpp>
#include <stdio.h>
#include <math.h>
#include <vp/mapping_tree.hpp>

#define GET_BITS(x, start, end) \
    (((x) >> (start)) & ((1U << ((end) - (start) + 1)) - 1))

static constexpr int REQ_SRC_ID = 0;
static constexpr int REQ_RESP_ADDR = 1;
static constexpr int REQ_BURST = 2;
static constexpr int REQ_NB_ARGS = 3;

class BandwidthLimiter;
class L1_XbarItf;
class InputPort;
class OutputPort;

class BandwidthLimiter
{
public:
    // Overall bandwidth to be respected, and global latency to be applied to each request
    BandwidthLimiter(OutputPort *top, int64_t bandwidth, int64_t latency, bool shared_rw_bandwidth);
    // Can be called on any request going through the limiter to add the fixed latency and impact
    // the latency and duration with the current utilization of the limiter with respect to the
    // bandwidth
    void apply_bandwidth(int64_t cycles, vp::IoReq *req);

private:
    OutputPort *top;
    // Bandwidth in bytes per cycle to be respected
    int64_t bandwidth;
    // Fixed latency to be added to each request
    int64_t latency;
    // Cyclestamp at which the next read burst can go through the limiter. Used to delay a request
    // which arrives before this cyclestamp.
    int64_t next_read_burst_cycle = 0;
    // Cyclestamp at which the write read burst can go through the limiter. Used to delay a request
    // which arrives before this cyclestamp.
    int64_t next_write_burst_cycle = 0;
    // Indicates whether the read and write share the bandwidth
    bool shared_rw_bandwidth = false;
};

/**
 * @brief OutputPort
 *
 * This represents a possible OutputPort in the memory map.
 * It is used to implement the bandwidth limiter, and store port information like request election
 * and stall state
 */
class OutputPort : public vp::Block
{
public:
    OutputPort(L1_XbarItf *top, std::string name, int64_t bandwidth, int64_t latency);
    // True when a request has been denied. The port can not send requests anymore until the
    // denied request is granted. This also stall the elected input port.
    BandwidthLimiter limiter;
    bool stalled = false;
    // When not NULL, indicates a request has been elected to be sent through this output port
    // and should be sent as soon as the bandwidth allows it. The input port can not send anymore
    // until this elected request is sent
    InputPort *elected_input = NULL;
    // Cycle stamp where the next request can be sent. It is updated each time a request is
    // sent, according to request and size and router bandwidth
    int64_t next_burst_cycle = 0;
    // When the output port is stalled, this indicates the input port where the request comes from
    // This allows unstalling the input port when the output port is unstalled since they are
    // stalled together
    InputPort *stalled_port;
    vp::Queue out_queue;
};

/**
 * @brief InputPort
 *
 * This represents a router input port, mostly used to store information about bandwidth limiter
 * and port state like pending requests
 */
class InputPort : public vp::Block
{
public:
    InputPort(int id, std::string name, L1_XbarItf *top, int64_t bandwidth, int64_t latency);
    int id;
    // Queue of pending requests. Any incoming request is first pushed here. An arbitration event
    // is then executed to route these requests to output ports
    std::queue<vp::IoReq *> pending_reqs;
    // Queue of requests which have been denied because the input FIFO size was full. As soon as
    // the FIFO becomes ready, requests are popped from this queue and pushed to the pending queue
    std::queue<vp::IoReq *> denied_reqs;
    // Input FIFO size. Incoming requests are denied as soon as this becomes equal or greater than
    // the FIFO size
    vp::Signal<int> pending_size;
    // If not NULL, this indicates the mapping tree information about the currently elected
    // request.
    vp::MappingTreeEntry *pending_mapping = NULL;
    // True if the input port is full because an elected request from this port was sent and denied.
    // The input port can still enqueue incoming requests unless the FIFO is full, but no more
    // requests are router until the port is unstalled
    bool stalled = false;
    // Cycle stamp where the next request can be elected. It is updated each time a request is
    // elected, according to request and size and router bandwidth
    int64_t next_burst_cycle = 0;
    // In case of a request crossing several mappings, this indicates the remaining size to be
    // processed
    uint64_t remaining_size = 0;
    // In case of a request crossing several mappings, this indicates the current request data
    uint8_t *current_data;
    // In case of a request crossing several mappings, this indicates the current request addr
    uint64_t current_addr;
};

class L1_XbarItf : public vp::Component
{
public:
    L1_XbarItf(vp::ComponentConf &conf);

private:
    static vp::IoReqStatus core_req(vp::Block *__this, vp::IoReq *req, int port);
    static vp::IoReqStatus noc_req(vp::Block *__this, vp::IoReq *req, int port);
    static vp::IoReqStatus noc_resp(vp::Block *__this, vp::IoReq *req, int port);

    static void tcdm_req_response(vp::Block *__this, vp::IoReq *req, int id);
    static void tcdm_req_grant(vp::Block *__this, vp::IoReq *req, int id);
    static void noc_req_response(vp::Block *__this, vp::IoReq *req, int id);
    static void noc_req_grant(vp::Block *__this, vp::IoReq *req, int id);
    static void noc_resp_response(vp::Block *__this, vp::IoReq *req, int id);
    static void noc_resp_grant(vp::Block *__this, vp::IoReq *req, int id);

    vp::IoReqStatus handle_core_req(vp::IoReq *req, int port);
    vp::IoReqStatus handle_noc_req(vp::IoReq *req, int port);
    vp::IoReqStatus handle_noc_resp(vp::IoReq *req, int port);

    vp::ClockEvent fsm_event;
    static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);

    int global_tile_id;
    uint64_t dummy_resp_addr;

    unsigned int clog2(int value);

    // This component trace
    vp::Trace trace;

    int nb_req_ports;
    int nb_resp_ports;

    std::vector<vp::IoSlave> core_req_slv_itfs;
    std::vector<vp::IoMaster *> tcdm_req_mst_itfs;

    std::vector<vp::IoMaster *> noc_req_mst_itfs;
    std::vector<vp::IoSlave> noc_resp_mst_itfs;

    std::vector<vp::IoSlave> noc_req_slv_itfs;
    std::vector<vp::IoMaster *> noc_resp_slv_itfs;

    std::vector<InputPort *> core_req_slvs;
    std::vector<OutputPort *> tcdm_req_msts;

    std::vector<OutputPort *> noc_req_msts;
    std::vector<InputPort *> noc_resp_msts;

    std::vector<InputPort *> noc_req_slvs;
    std::vector<OutputPort *> noc_resp_slvs;
};

L1_XbarItf::L1_XbarItf(vp::ComponentConf &config)
    : vp::Component(config), fsm_event(this, L1_XbarItf::fsm_handler)
{
    this->traces.new_trace("trace", &trace, vp::DEBUG);

    this->nb_req_ports = this->get_js_config()->get_int("nb_req_ports");
    this->nb_resp_ports = this->get_js_config()->get_int("nb_resp_ports");

    int tile_id = this->get_js_config()->get_int("tile_id");
    int sub_group_id = this->get_js_config()->get_int("sub_group_id");
    int group_id = this->get_js_config()->get_int("group_id");
    int nb_tiles_per_sub_group = this->get_js_config()->get_int("nb_tiles_per_sub_group");
    int nb_sub_groups_per_group = this->get_js_config()->get_int("nb_sub_groups_per_group");
    this->global_tile_id = tile_id + sub_group_id * nb_tiles_per_sub_group + group_id * nb_sub_groups_per_group * nb_tiles_per_sub_group;

    int byte_offset = this->get_js_config()->get_int("byte_offset");
    int num_banks_per_tile = this->get_js_config()->get_int("num_banks_per_tile");
    this->dummy_resp_addr = this->global_tile_id * num_banks_per_tile * (1 << byte_offset);

    this->core_req_slv_itfs.resize(nb_req_ports);
    this->core_req_slvs.resize(nb_req_ports);
    for (int i = 0; i < nb_req_ports; i++)
    {
        vp::IoSlave *input = &this->core_req_slv_itfs[i];
        input->set_req_meth_muxed(&L1_XbarItf::core_req, i);
        this->new_slave_port("core_req_slv_" + std::to_string(i), input, this);
        this->core_req_slvs[i] = new InputPort(i, "core_req_slv_" + std::to_string(i), this, 4, 0);
    }
    this->tcdm_req_mst_itfs.resize(nb_req_ports);
    this->tcdm_req_msts.resize(nb_req_ports);
    for (int i = 0; i < nb_req_ports; i++)
    {
        vp::IoMaster *output = new vp::IoMaster();
        this->tcdm_req_mst_itfs[i] = output;
        this->new_master_port("tcdm_req_mst_" + std::to_string(i), output, this);
        output->set_resp_meth_muxed(&L1_XbarItf::tcdm_req_response, i);
        output->set_grant_meth_muxed(&L1_XbarItf::tcdm_req_grant, i);
        this->tcdm_req_msts[i] = new OutputPort(this, "tcdm_req_mst_" + std::to_string(i), 4, 0);
    }
    this->noc_req_mst_itfs.resize(nb_req_ports);
    this->noc_req_msts.resize(nb_req_ports);
    for (int i = 0; i < nb_req_ports; i++)
    {
        vp::IoMaster *output = new vp::IoMaster();
        this->noc_req_mst_itfs[i] = output;
        this->new_master_port("noc_req_mst_" + std::to_string(i), output, this);
        output->set_resp_meth_muxed(&L1_XbarItf::noc_req_response, i);
        output->set_grant_meth_muxed(&L1_XbarItf::noc_req_grant, i);
        this->noc_req_msts[i] = new OutputPort(this, "noc_req_mst_" + std::to_string(i), 4, 0);
    }
    this->noc_resp_mst_itfs.resize(nb_resp_ports);
    this->noc_resp_msts.resize(nb_resp_ports);
    for (int i = 0; i < nb_resp_ports; i++)
    {
        vp::IoSlave *input = &this->noc_resp_mst_itfs[i];
        input->set_req_meth_muxed(&L1_XbarItf::noc_resp, i);
        this->new_slave_port("noc_resp_mst_" + std::to_string(i), input, this);
        this->noc_resp_msts[i] = new InputPort(i, "noc_resp_mst_" + std::to_string(i), this, 4, 0);
    }
    this->noc_req_slv_itfs.resize(nb_req_ports);
    this->noc_req_slvs.resize(nb_req_ports);
    for (int i = 0; i < nb_req_ports; i++)
    {
        vp::IoSlave *input = &this->noc_req_slv_itfs[i];
        input->set_req_meth_muxed(&L1_XbarItf::noc_req, i);
        this->new_slave_port("noc_req_slv_" + std::to_string(i), input, this);
        this->noc_req_slvs[i] = new InputPort(i, "noc_req_slv_" + std::to_string(i), this, 4, 0);
    }
    this->noc_resp_slv_itfs.resize(nb_resp_ports);
    this->noc_resp_slvs.resize(nb_resp_ports);
    for (int i = 0; i < nb_resp_ports; i++)
    {
        vp::IoMaster *output = new vp::IoMaster();
        this->noc_resp_slv_itfs[i] = output;
        this->new_master_port("noc_resp_slv_" + std::to_string(i), output, this);
        output->set_resp_meth_muxed(&L1_XbarItf::noc_resp_response, i);
        output->set_grant_meth_muxed(&L1_XbarItf::noc_resp_grant, i);
        this->noc_resp_slvs[i] = new OutputPort(this, "noc_resp_slv_" + std::to_string(i), 4, 1);
    }
}

vp::IoReqStatus L1_XbarItf::core_req(vp::Block *__this, vp::IoReq *req, int port)
{
    L1_XbarItf *_this = (L1_XbarItf *)__this;
    return _this->handle_core_req(req, port);
}

vp::IoReqStatus L1_XbarItf::noc_req(vp::Block *__this, vp::IoReq *req, int port)
{
    L1_XbarItf *_this = (L1_XbarItf *)__this;
    return _this->handle_noc_req(req, port);
}

vp::IoReqStatus L1_XbarItf::noc_resp(vp::Block *__this, vp::IoReq *req, int port)
{
    L1_XbarItf *_this = (L1_XbarItf *)__this;
    return _this->handle_noc_resp(req, port);
}

void L1_XbarItf::tcdm_req_response(vp::Block *__this, vp::IoReq *req, int id)
{
    L1_XbarItf *_this = (L1_XbarItf *)__this;
    int64_t cycles = _this->clock.get_cycles();
    // _this->noc_req_slvs[id]->stalled = false;
    vp::IoSlave *core_resp_port = (vp::IoSlave *)req->arg_pop();
    vp::IoReq *noc_req = (vp::IoReq *)req->arg_pop();
    req->resp_port = core_resp_port;
    noc_req->set_addr((uint64_t)*req->arg_get(REQ_RESP_ADDR));
    req->reset_latency();
    _this->noc_resp_slvs[id]->limiter.apply_bandwidth(cycles, req);
    int latency = req->get_latency();
    _this->noc_resp_slvs[id]->out_queue.push_back(noc_req, latency > 0 ? latency - 1 : 0);
    _this->fsm_event.enqueue();
}

void L1_XbarItf::tcdm_req_grant(vp::Block *__this, vp::IoReq *req, int id)
{
    L1_XbarItf *_this = (L1_XbarItf *)__this;
    _this->noc_req_slvs[id]->stalled = false;
    _this->fsm_event.enqueue();
}

void L1_XbarItf::noc_req_response(vp::Block *__this, vp::IoReq *req, int id)
{
    L1_XbarItf *_this = (L1_XbarItf *)__this;
    _this->trace.fatal("L1_XbarItf: noc_req_response should not be called\n");
}

void L1_XbarItf::noc_req_grant(vp::Block *__this, vp::IoReq *req, int id)
{
    L1_XbarItf *_this = (L1_XbarItf *)__this;
    _this->noc_req_msts[id]->stalled = false;
    vp::IoReq *core_req = (vp::IoReq *)*req->arg_get(REQ_BURST);
    core_req->resp_port->grant(core_req);

    _this->fsm_event.enqueue();

    // if (_this->noc_req_msts[id]->stalled_port)
    // {
    //     _this->noc_req_msts[id]->stalled_port->stalled = false;
    //     _this->noc_req_msts[id]->stalled_port = NULL;
    // }
}

void L1_XbarItf::noc_resp_response(vp::Block *__this, vp::IoReq *req, int id)
{
    L1_XbarItf *_this = (L1_XbarItf *)__this;
    _this->trace.fatal("L1_XbarItf: noc_resp_response should not be called\n");
}

void L1_XbarItf::noc_resp_grant(vp::Block *__this, vp::IoReq *req, int id)
{
    L1_XbarItf *_this = (L1_XbarItf *)__this;
    _this->noc_resp_slvs[id]->stalled = false;

    _this->fsm_event.enqueue();
}

vp::IoReqStatus L1_XbarItf::handle_core_req(vp::IoReq *req, int port)
{
    this->trace.msg(vp::Trace::LEVEL_TRACE, "L1_XbarItf: core_req port: %d addr: 0x%x size: %d opcode: %d\n", port, req->get_addr(), req->get_size(), req->get_opcode());

    int64_t cycles = this->clock.get_cycles();
    if (noc_req_msts[port]->stalled || cycles < core_req_slvs[port]->next_burst_cycle || core_req_slvs[port]->denied_reqs.size() > 0)
    {
        core_req_slvs[port]->denied_reqs.push(req);
        this->fsm_event.enqueue();
        return vp::IO_REQ_DENIED;
    }

    vp::IoReq *noc_req = new vp::IoReq();
    noc_req->init();
    noc_req->arg_alloc(REQ_NB_ARGS);
    *noc_req->arg_get(REQ_SRC_ID) = (void *)(long)(this->global_tile_id);
    *noc_req->arg_get(REQ_RESP_ADDR) = (void *)(long)(this->dummy_resp_addr);
    *noc_req->arg_get(REQ_BURST) = (void *)req;
    noc_req->set_addr(req->get_addr());
    noc_req->set_size(req->get_size());

    core_req_slvs[port]->next_burst_cycle = cycles + 1;
    vp::IoReqStatus retval = noc_req_mst_itfs[port]->req(noc_req);
    if (retval == vp::IO_REQ_DENIED)
    {
        // core_req_slvs[port]->stalled = true;
        noc_req_msts[port]->stalled = true;
        // noc_req_msts[port]->stalled_port = core_req_slvs[port];
    }
    return retval;
}

vp::IoReqStatus L1_XbarItf::handle_noc_req(vp::IoReq *req, int port)
{
    this->trace.msg(vp::Trace::LEVEL_TRACE, "L1_XbarItf: noc_req port: %d src_tile: %d\n", port, (long)*req->arg_get(REQ_SRC_ID));

    int64_t cycles = this->clock.get_cycles();
    if (!noc_req_slvs[port]->stalled && cycles < noc_req_slvs[port]->next_burst_cycle || noc_req_slvs[port]->denied_reqs.size() > 0)
    {
        noc_req_slvs[port]->denied_reqs.push(req);
        this->fsm_event.enqueue();
        return vp::IO_REQ_DENIED;
    }

    vp::IoReq *core_req = (vp::IoReq *)*req->arg_get(REQ_BURST);
    vp::IoSlave *core_resp_port = core_req->resp_port;

    this->trace.msg(vp::Trace::LEVEL_TRACE, "L1_XbarItf: noc_req translate to tcdm_req addr: 0x%x size: %d opcode: %d\n", core_req->get_addr(), core_req->get_size(), core_req->get_opcode());

    noc_req_slvs[port]->next_burst_cycle = cycles + 1;
    core_req->arg_push((void *)req);
    core_req->arg_push((void *)core_resp_port);
    vp::IoReqStatus retval = this->tcdm_req_mst_itfs[port]->req(core_req);

    if (retval == vp::IO_REQ_OK)
    {
        core_req->arg_pop();
        core_req->arg_pop();
        core_req->resp_port = core_resp_port;
        req->set_addr((uint64_t)*req->arg_get(REQ_RESP_ADDR));
        this->noc_resp_slvs[port]->limiter.apply_bandwidth(cycles, core_req);
        int latency = core_req->get_latency();
        this->noc_resp_slvs[port]->out_queue.push_back(req, latency > 0 ? latency - 1 : 0);
        if (latency > 2)
        {
            noc_req_slvs[port]->next_burst_cycle += (latency - 2);
        }
        this->fsm_event.enqueue();
        this->trace.msg(vp::Trace::LEVEL_TRACE, "L1_XbarItf: noc_resp will be sent after %d cycles\n", latency);
    }
    else if (retval == vp::IO_REQ_DENIED)
    {
        noc_req_slvs[port]->stalled = true;
    }
    return vp::IO_REQ_PENDING;
}

vp::IoReqStatus L1_XbarItf::handle_noc_resp(vp::IoReq *req, int port)
{
    this->trace.msg(vp::Trace::LEVEL_TRACE, "L1_XbarItf: noc_resp port: %d\n", port);

    vp::IoReq *core_req = (vp::IoReq *)*req->arg_get(REQ_BURST);

    this->trace.msg(vp::Trace::LEVEL_TRACE, "L1_XbarItf: noc_resp translate to core_resp addr: 0x%x size: %d opcode: %d\n", core_req->get_addr(), core_req->get_size(), core_req->get_opcode());

    core_req->resp_port->resp(core_req);
    delete req;
    return vp::IO_REQ_PENDING;
}

void L1_XbarItf::fsm_handler(vp::Block *__this, vp::ClockEvent *event)
{
    L1_XbarItf *_this = (L1_XbarItf *)__this;
    int64_t cycles = _this->clock.get_cycles();

    for (int i = 0; i < _this->nb_req_ports; i++)
    {
        InputPort *input = _this->core_req_slvs[i];
        OutputPort *output = _this->noc_req_msts[i];
        vp::IoMaster *output_itf = _this->noc_req_mst_itfs[i];
        if (!output->stalled && cycles >= input->next_burst_cycle && input->denied_reqs.size() > 0)
        {
            vp::IoReq *req = input->denied_reqs.front();
            input->denied_reqs.pop();

            _this->trace.msg(vp::Trace::LEVEL_TRACE, "L1_XbarItf fsm: core_req addr: 0x%x size: %d opcode: %d\n", req->get_addr(), req->get_size(), req->get_opcode());

            vp::IoReq *noc_req = new vp::IoReq();
            noc_req->init();
            noc_req->arg_alloc(REQ_NB_ARGS);
            *noc_req->arg_get(REQ_SRC_ID) = (void *)(long)(_this->global_tile_id);
            *noc_req->arg_get(REQ_RESP_ADDR) = (void *)(long)(_this->dummy_resp_addr);
            *noc_req->arg_get(REQ_BURST) = (void *)req;
            noc_req->set_addr(req->get_addr());
            noc_req->set_size(req->get_size());

            input->next_burst_cycle = cycles + 1;
            vp::IoReqStatus retval = output_itf->req(noc_req);
            if (retval == vp::IO_REQ_DENIED)
            {
                // input->stalled = true;
                output->stalled = true;
                // output->stalled_port = input;
            }
            else
            {
                req->resp_port->grant(req);
                _this->fsm_event.enqueue();
            }
        }
        if (input->denied_reqs.size() > 0)
        {
            _this->fsm_event.enqueue();
        }
    }

    for (int i = 0; i < _this->nb_req_ports; i++)
    {
        InputPort *input = _this->noc_req_slvs[i];
        OutputPort *output = _this->noc_resp_slvs[i];
        vp::IoMaster *tcdm_itf = _this->tcdm_req_mst_itfs[i];
        if (!input->stalled && cycles >= input->next_burst_cycle && input->denied_reqs.size() > 0)
        {
            vp::IoReq *req = input->denied_reqs.front();
            input->denied_reqs.pop();
            req->resp_port->grant(req);

            vp::IoReq *core_req = (vp::IoReq *)*req->arg_get(REQ_BURST);
            vp::IoSlave *core_resp_port = core_req->resp_port;

            _this->trace.msg(vp::Trace::LEVEL_TRACE, "L1_XbarItf fsm: noc_req translate to tcdm_req addr: 0x%x size: %d opcode: %d\n", core_req->get_addr(), core_req->get_size(), core_req->get_opcode());

            input->next_burst_cycle = cycles + 1;
            core_req->arg_push((void *)req);
            core_req->arg_push((void *)core_resp_port);
            vp::IoReqStatus retval = tcdm_itf->req(core_req);

            if (retval == vp::IO_REQ_OK)
            {
                core_req->arg_pop();
                core_req->arg_pop();
                core_req->resp_port = core_resp_port;
                req->set_addr((uint64_t)*req->arg_get(REQ_RESP_ADDR));
                output->limiter.apply_bandwidth(cycles, core_req);
                int latency = core_req->get_latency();
                output->out_queue.push_back(req, latency > 0 ? latency - 1 : 0);
                if (latency > 2)
                {
                    input->next_burst_cycle += (latency - 2);
                }
                _this->fsm_event.enqueue();
                _this->trace.msg(vp::Trace::LEVEL_TRACE, "L1_XbarItf fsm: noc_resp will be sent after %d cycles\n", latency);
            }
            else if (retval == vp::IO_REQ_DENIED)
            {
                input->stalled = true;
            }
        }
        if (input->denied_reqs.size() > 0)
        {
            _this->fsm_event.enqueue();
        }
    }

    for (int i = 0; i < _this->nb_resp_ports; i++)
    {
        OutputPort *output = _this->noc_resp_slvs[i];
        vp::IoMaster *output_itf = _this->noc_resp_slv_itfs[i];
        if (!output->stalled && !output->out_queue.empty())
        {
            vp::IoReq *noc_req = (vp::IoReq *)output->out_queue.pop();
            vp::IoReqStatus retval = output_itf->req(noc_req);
            if (retval == vp::IO_REQ_DENIED)
            {
                output->stalled = true;
            }
        }
        if (output->out_queue.size() > 0)
        {
            _this->fsm_event.enqueue();
        }
    }
}

unsigned int L1_XbarItf::clog2(int value)
{
    unsigned int result = 0;
    value--;
    while (value > 0) {
        value >>= 1;
        result++;
    }
    return result;
}

// vp::IoReqStatus L1_XbarItf::handle_req(vp::IoReq *req, int port)
// {
//     uint64_t offset = req->get_addr();
//     uint64_t size = req->get_size();
//     uint8_t *data = req->get_data();
//     bool is_write = req->get_is_write();

//     this->trace.msg(vp::Trace::LEVEL_TRACE, "Received IO req (offset: 0x%llx, size: 0x%llx, is_write: %d)\n",
//         offset, size, is_write);

//     // The mapping may exist and not be connected, return an error in this case
//     if (!this->output_itf.is_bound())
//     {
//         this->trace.fatal("L1_XbarItf: output port is not connected\n");
//         return vp::IO_REQ_INVALID;
//     }

//     // Foward the request to the output port
//     vp::IoReqStatus retval = this->output_itf.req_forward(req);

//     return retval;
// }

OutputPort::OutputPort(L1_XbarItf *top, std::string name, int64_t bandwidth, int64_t latency)
: vp::Block(top, name), out_queue(this, "out_queue"), limiter(this, bandwidth, latency, false)
{
}

InputPort::InputPort(int id, std::string name, L1_XbarItf *top, int64_t bandwidth, int64_t latency)
: vp::Block(top, name), id(id), pending_size(*top, name + "/pending_size", 32, vp::SignalCommon::ResetKind::Value, 0)
{
}

BandwidthLimiter::BandwidthLimiter(OutputPort *top, int64_t bandwidth, int64_t latency, bool shared_rw_bandwidth)
{
    this->top = top;
    this->latency = latency;
    this->bandwidth = bandwidth;
    this->shared_rw_bandwidth = shared_rw_bandwidth;
}

void BandwidthLimiter::apply_bandwidth(int64_t cycles, vp::IoReq *req)
{
    uint64_t size = req->get_size();

    if (this->bandwidth != 0)
    {
        // Bandwidth was specified

        // Duration in cycles of this burst in this router according to router bandwidth
        int64_t burst_duration = (size + this->bandwidth - 1) / this->bandwidth;

        // Update burst duration
        // This will update it only if it is bigger than the current duration, in case there is a
        // slower router on the path
        req->set_duration(burst_duration);

        // Now we need to compute the start cycle of the burst, which is its latency.
        // First get the cyclestamp where the router becomes available, due to previous requests
        int64_t *next_burst_cycle = (req->get_is_write() || this->shared_rw_bandwidth) ?
            &this->next_write_burst_cycle : &this->next_read_burst_cycle;
        int64_t router_latency = *next_burst_cycle - cycles;

        // Then compare that to the request latency and take the highest to properly delay the
        // request in case the bandwidth is reached.
        int64_t latency = std::max((int64_t)req->get_latency(), router_latency);

        // Apply the computed latency and add the fixed one
        req->set_latency(latency + this->latency);

        // Update the bandwidth information by appending the new burst right after the previous one.
        *next_burst_cycle = std::max(cycles, *next_burst_cycle) + burst_duration;

    }
    else
    {
        // No bandwidth was specified, just add the specified latency
        req->inc_latency(this->latency);
    }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new L1_XbarItf(config);
}
