// Verilated -*- C++ -*-
// DESCRIPTION: Verilator output: Design implementation internals
// See VVortex.h for the primary calling header

#include "VVortex.h"
#include "VVortex__Syms.h"


//--------------------
// STATIC VARIABLES

// Begin mtask footprint  all: 
VL_ST_SIG8(VVortex::__Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu[8],4,0);

//--------------------

VL_CTOR_IMP(VVortex) {
    VVortex__Syms* __restrict vlSymsp = __VlSymsp = new VVortex__Syms(this, name());
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    VL_CELL(__PVT__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one, VVortex_VX_context_slave);
    VL_CELL(__PVT__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one, VVortex_VX_context_slave);
    VL_CELL(__PVT__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one, VVortex_VX_context_slave);
    VL_CELL(__PVT__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one, VVortex_VX_context_slave);
    VL_CELL(__PVT__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one, VVortex_VX_context_slave);
    VL_CELL(__PVT__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one, VVortex_VX_context_slave);
    VL_CELL(__PVT__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one, VVortex_VX_context_slave);
    // Reset internal values
    
    // Reset structure values
    _ctor_var_reset();
}

void VVortex::__Vconfigure(VVortex__Syms* vlSymsp, bool first) {
    if (0 && first) {}  // Prevent unused
    this->__VlSymsp = vlSymsp;
}

VVortex::~VVortex() {
    delete __VlSymsp; __VlSymsp=NULL;
}

//--------------------


void VVortex::eval() {
    VL_DEBUG_IF(VL_DBG_MSGF("+++++TOP Evaluate VVortex::eval\n"); );
    VVortex__Syms* __restrict vlSymsp = this->__VlSymsp;  // Setup global symbol table
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
#ifdef VL_DEBUG
    // Debug assertions
    _eval_debug_assertions();
#endif // VL_DEBUG
    // Initialize
    if (VL_UNLIKELY(!vlSymsp->__Vm_didInit)) _eval_initial_loop(vlSymsp);
    // Evaluate till stable
    int __VclockLoop = 0;
    QData __Vchange = 1;
    do {
	VL_DEBUG_IF(VL_DBG_MSGF("+ Clock loop\n"););
	_eval(vlSymsp);
	if (VL_UNLIKELY(++__VclockLoop > 100)) {
	    // About to fail, so enable debug to see what's not settling.
	    // Note you must run make with OPT=-DVL_DEBUG for debug prints.
	    int __Vsaved_debug = Verilated::debug();
	    Verilated::debug(1);
	    __Vchange = _change_request(vlSymsp);
	    Verilated::debug(__Vsaved_debug);
	    VL_FATAL_MT(__FILE__,__LINE__,__FILE__,"Verilated model didn't converge");
	} else {
	    __Vchange = _change_request(vlSymsp);
	}
    } while (VL_UNLIKELY(__Vchange));
}

void VVortex::_eval_initial_loop(VVortex__Syms* __restrict vlSymsp) {
    vlSymsp->__Vm_didInit = true;
    _eval_initial(vlSymsp);
    // Evaluate till stable
    int __VclockLoop = 0;
    QData __Vchange = 1;
    do {
	_eval_settle(vlSymsp);
	_eval(vlSymsp);
	if (VL_UNLIKELY(++__VclockLoop > 100)) {
	    // About to fail, so enable debug to see what's not settling.
	    // Note you must run make with OPT=-DVL_DEBUG for debug prints.
	    int __Vsaved_debug = Verilated::debug();
	    Verilated::debug(1);
	    __Vchange = _change_request(vlSymsp);
	    Verilated::debug(__Vsaved_debug);
	    VL_FATAL_MT(__FILE__,__LINE__,__FILE__,"Verilated model didn't DC converge");
	} else {
	    __Vchange = _change_request(vlSymsp);
	}
    } while (VL_UNLIKELY(__Vchange));
}

//--------------------
// Internal Methods

void VVortex::_initial__TOP__1(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_initial__TOP__1\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    // INITIAL at VX_warp.v:30
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__real_PC = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid_zero[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid_zero[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid_zero[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[0U] = 1U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid_zero[0U] = 0U;
    // INITIAL at VX_warp.v:30
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__real_PC = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid_zero[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid_zero[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid_zero[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[0U] = 1U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid_zero[0U] = 0U;
    // INITIAL at VX_warp.v:30
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__real_PC = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid_zero[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid_zero[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid_zero[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[0U] = 1U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid_zero[0U] = 0U;
    // INITIAL at VX_warp.v:30
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__real_PC = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid_zero[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid_zero[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid_zero[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[0U] = 1U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid_zero[0U] = 0U;
    // INITIAL at VX_warp.v:30
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__real_PC = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid_zero[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid_zero[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid_zero[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[0U] = 1U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid_zero[0U] = 0U;
    // INITIAL at VX_warp.v:30
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__real_PC = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid_zero[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid_zero[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid_zero[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[0U] = 1U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid_zero[0U] = 0U;
    // INITIAL at VX_warp.v:30
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__real_PC = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid_zero[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid_zero[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid_zero[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[0U] = 1U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid_zero[0U] = 0U;
    // INITIAL at VX_warp.v:30
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__real_PC = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid_zero[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid_zero[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid_zero[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[0U] = 1U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid_zero[0U] = 0U;
    // INITIAL at VX_context.v:31
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall = 0U;
    // INITIAL at VX_m_w_reg.v:41
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd = 0U;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb = 0U;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next = 0U;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__warp_num = 0U;
    // INITIAL at VX_csr_handler.v:29
    vlTOPp->Vortex__DOT__vx_csr_handler__DOT__cycle = VL_ULL(0);
    vlTOPp->Vortex__DOT__vx_csr_handler__DOT__instret = VL_ULL(0);
    vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address = 0U;
    // INITIAL at VX_e_m_reg.v:79
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__rd = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__mem_read = 7U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__mem_write = 7U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__csr_address = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__is_csr = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__csr_result = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__curr_PC = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_offset = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__a_reg_data[0U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data[0U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid[0U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result[0U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__a_reg_data[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__a_reg_data[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__a_reg_data[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result[3U] = 0U;
    // INITIAL at VX_d_e_reg.v:87
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rd = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data[0U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data[0U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z[0U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid[0U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid_z[0U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid_z[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid_z[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid_z[3U] = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rs2_src = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__mem_read = 7U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__mem_write = 7U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__branch_type = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__csr_address = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__is_csr = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__csr_mask = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__jal = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__jal_offset = 0U;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__warp_num = 0U;
    // INITIAL at VX_fetch.v:47
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state = 0U;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_count = 1U;
}

void VVortex::_settle__TOP__2(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_settle__TOP__2\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    vlTOPp->Vortex__DOT__vx_decode__DOT__jmprt_thread_mask[0U] = 1U;
    vlTOPp->Vortex__DOT__vx_decode__DOT__jmprt_thread_mask[1U] = 0U;
    vlTOPp->Vortex__DOT__vx_decode__DOT__jmprt_thread_mask[2U] = 0U;
    vlTOPp->Vortex__DOT__vx_decode__DOT__jmprt_thread_mask[3U] = 0U;
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data[3U] 
	= vlTOPp->in_cache_driver_out_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data[2U] 
	= vlTOPp->in_cache_driver_out_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data[1U] 
	= vlTOPp->in_cache_driver_out_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data[0U] 
	= vlTOPp->in_cache_driver_out_data[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_src1_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__out_src1_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__out_src1_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__out_src1_data;
    vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_src2_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__out_src2_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__out_src2_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__out_src2_data;
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result[3U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result[2U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result[1U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result[0U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result[3U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result[2U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result[1U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result[0U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_itype = 
	((0x13U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	 | (3U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)));
    vlTOPp->Vortex__DOT__decode_csr_address = (0xfffU 
					       & (((0U 
						    != 
						    (7U 
						     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							>> 0xcU))) 
						   & (2U 
						      <= 
						      (0xfffU 
						       & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							  >> 0x14U))))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						   >> 0x14U)
						   : 0x55U));
    // ALWAYS at VX_decode.v:577
    vlTOPp->__Vtableidx1 = (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				  >> 0xcU));
    vlTOPp->Vortex__DOT__vx_decode__DOT__mul_alu = 
	vlTOPp->__Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu
	[vlTOPp->__Vtableidx1];
    vlTOPp->Vortex__DOT__vx_decode__DOT__alu_tempp 
	= (0xfffU & (((1U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				    >> 0xcU))) | (5U 
						  == 
						  (7U 
						   & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						      >> 0xcU))))
		      ? (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				  >> 0x14U)) : (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						>> 0x14U)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_csr = (
						   (0x73U 
						    == 
						    (0x7fU 
						     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
						   & (0U 
						      != 
						      (7U 
						       & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							  >> 0xcU))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__jal_sys_jal 
	= ((0U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0xcU))) & (2U > (0xfffU 
					     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						>> 0x14U))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt = 
	((0x6bU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	 & (4U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0xcU))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn 
	= ((0x6bU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	   & (0U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			   >> 0xcU))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone = 
	((0x6bU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	 & (5U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0xcU))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs = 
	((0x6bU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	 & (6U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0xcU))));
    vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data[0U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data[1U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data[2U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data[3U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next[0U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next[1U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next[2U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next[3U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__csr_decode_csr_data = ((0xc00U 
						 == (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address))
						 ? (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__cycle)
						 : 
						((0xc80U 
						  == (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address))
						  ? (IData)(
							    (vlTOPp->Vortex__DOT__vx_csr_handler__DOT__cycle 
							     >> 0x20U))
						  : 
						 ((0xc02U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address))
						   ? (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__instret)
						   : 
						  ((0xc82U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address))
						    ? (IData)(
							      (vlTOPp->Vortex__DOT__vx_csr_handler__DOT__instret 
							       >> 0x20U))
						    : 
						   ((0x400U 
						     >= 
						     (0x7ffU 
						      & (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address)))
						     ? 
						    vlTOPp->Vortex__DOT__vx_csr_handler__DOT__csr
						    [
						    (0x7ffU 
						     & (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address))]
						     : 0U)))));
    vlTOPp->out_cache_driver_in_mem_read = vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__mem_read;
    vlTOPp->out_cache_driver_in_mem_write = vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__mem_write;
    vlTOPp->Vortex__DOT__memory_branch_dest = (vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__curr_PC 
					       + (vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_offset 
						  << 1U));
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid
	[0U];
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next[0U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next[1U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next[2U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next[3U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result[3U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result[2U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result[1U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result[0U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid
	[0U];
    vlTOPp->Vortex__DOT__execute_branch_stall = ((0U 
						  != (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__branch_type)) 
						 | (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__jal));
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next[0U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next[1U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next[2U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next[3U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out;
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data[3U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data[2U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data[1U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data[0U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd 
	= (((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			>> 0xfU)) == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rd)) 
	     & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				>> 0xfU)))) & (0U != (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num) 
	      == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd 
	= (((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			>> 0x14U)) == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rd)) 
	     & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				>> 0x14U)))) & (0U 
						!= (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num) 
	      == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data
	[0U];
    vlTOPp->Vortex__DOT__m_w_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid
	[3U];
    vlTOPp->Vortex__DOT__m_w_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid
	[2U];
    vlTOPp->Vortex__DOT__m_w_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid
	[1U];
    vlTOPp->Vortex__DOT__m_w_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid
	[0U];
    vlTOPp->Vortex__DOT__f_d_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid
	[3U];
    vlTOPp->Vortex__DOT__f_d_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid
	[2U];
    vlTOPp->Vortex__DOT__f_d_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid
	[1U];
    vlTOPp->Vortex__DOT__f_d_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0U];
    vlTOPp->Vortex__DOT__m_w_alu_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result
	[3U];
    vlTOPp->Vortex__DOT__m_w_alu_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result
	[2U];
    vlTOPp->Vortex__DOT__m_w_alu_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result
	[1U];
    vlTOPp->Vortex__DOT__m_w_alu_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result
	[0U];
    vlTOPp->Vortex__DOT__m_w_mem_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result
	[3U];
    vlTOPp->Vortex__DOT__m_w_mem_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result
	[2U];
    vlTOPp->Vortex__DOT__m_w_mem_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result
	[1U];
    vlTOPp->Vortex__DOT__m_w_mem_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result
	[0U];
    // ALWAYS at VX_decode.v:508
    vlTOPp->Vortex__DOT__decode_itype_immed = ((0x40U 
						& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
					        ? (
						   (0x20U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 
						   ((0x10U 
						     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						     ? 0xdeadbeefU
						     : 
						    ((8U 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						      ? 0xdeadbeefU
						      : 
						     ((4U 
						       & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						       ? 0xdeadbeefU
						       : 
						      ((2U 
							& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						        ? 
						       ((1U 
							 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
							 ? 
							((0xfffff000U 
							  & (VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
										>> 0x1fU)))) 
							     << 0xcU)) 
							 | ((0x800U 
							     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								>> 0x14U)) 
							    | ((0x400U 
								& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								   << 3U)) 
							       | ((0x3f0U 
								   & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								      >> 0x15U)) 
								  | (0xfU 
								     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
									>> 8U))))))
							 : 0xdeadbeefU)
						        : 0xdeadbeefU))))
						    : 0xdeadbeefU)
					        : (
						   (0x20U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 
						   ((0x10U 
						     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						     ? 0xdeadbeefU
						     : 
						    ((8U 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						      ? 0xdeadbeefU
						      : 
						     ((4U 
						       & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						       ? 0xdeadbeefU
						       : 
						      ((2U 
							& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						        ? 
						       ((1U 
							 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
							 ? 
							((0xfffff000U 
							  & (VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
										>> 0x1fU)))) 
							     << 0xcU)) 
							 | ((0xfe0U 
							     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								>> 0x14U)) 
							    | (0x1fU 
							       & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								  >> 7U))))
							 : 0xdeadbeefU)
						        : 0xdeadbeefU))))
						    : 
						   ((0x10U 
						     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						     ? 
						    ((8U 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						      ? 0xdeadbeefU
						      : 
						     ((4U 
						       & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						       ? 0xdeadbeefU
						       : 
						      ((2U 
							& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						        ? 
						       ((1U 
							 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
							 ? 
							((0xfffff000U 
							  & (VL_NEGATE_I((IData)(
										(1U 
										& ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__alu_tempp) 
										>> 0xbU)))) 
							     << 0xcU)) 
							 | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__alu_tempp))
							 : 0xdeadbeefU)
						        : 0xdeadbeefU)))
						     : 
						    ((8U 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						      ? 0xdeadbeefU
						      : 
						     ((4U 
						       & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						       ? 0xdeadbeefU
						       : 
						      ((2U 
							& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						        ? 
						       ((1U 
							 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
							 ? 
							((0xfffff000U 
							  & (VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
										>> 0x1fU)))) 
							     << 0xcU)) 
							 | (0xfffU 
							    & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							       >> 0x14U)))
							 : 0xdeadbeefU)
						        : 0xdeadbeefU))))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (1U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (2U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (3U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (4U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (5U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (6U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (7U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (1U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (2U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (3U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (4U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (5U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (6U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (7U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    // ALWAYS at VX_decode.v:451
    if ((0x40U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	if ((0x20U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	    if ((0x10U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		vlTOPp->Vortex__DOT__decode_jal_offset 
		    = ((8U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
		        ? 0xdeadbeefU : ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
					  ? 0xdeadbeefU
					  : ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
					      ? ((1U 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						  ? 
						 (((0U 
						    == 
						    (7U 
						     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							>> 0xcU))) 
						   & (2U 
						      > 
						      (0xfffU 
						       & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							  >> 0x14U))))
						   ? 0xb0000000U
						   : 0xdeadbeefU)
						  : 0xdeadbeefU)
					      : 0xdeadbeefU)));
	    } else {
		if ((8U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		    if ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			vlTOPp->Vortex__DOT__decode_jal_offset 
			    = ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
			        ? ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
				    ? ((0xffe00000U 
					& (VL_NEGATE_I((IData)(
							       (1U 
								& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								   >> 0x1fU)))) 
					   << 0x15U)) 
				       | ((0x100000U 
					   & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					      >> 0xbU)) 
					  | ((0xff000U 
					      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction) 
					     | ((0x800U 
						 & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						    >> 9U)) 
						| (0x7feU 
						   & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						      >> 0x14U))))))
				    : 0xdeadbeefU) : 0xdeadbeefU);
		    } else {
			if ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			    if ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
				if (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs) 
				     | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt))) {
				    vlTOPp->Vortex__DOT__decode_jal_offset = 0U;
				}
			    } else {
				vlTOPp->Vortex__DOT__decode_jal_offset = 0xdeadbeefU;
			    }
			} else {
			    vlTOPp->Vortex__DOT__decode_jal_offset = 0xdeadbeefU;
			}
		    }
		} else {
		    vlTOPp->Vortex__DOT__decode_jal_offset 
			= ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
			    ? ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
			        ? ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
				    ? ((0xfffff000U 
					& (VL_NEGATE_I((IData)(
							       (1U 
								& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								   >> 0x1fU)))) 
					   << 0xcU)) 
				       | (0xfffU & 
					  (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					   >> 0x14U)))
				    : 0xdeadbeefU) : 0xdeadbeefU)
			    : 0xdeadbeefU);
		}
	    }
	} else {
	    vlTOPp->Vortex__DOT__decode_jal_offset = 0xdeadbeefU;
	}
    } else {
	vlTOPp->Vortex__DOT__decode_jal_offset = 0xdeadbeefU;
    }
    // ALWAYS at VX_decode.v:519
    if ((0x40U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	if ((0x20U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	    if ((0x10U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		vlTOPp->Vortex__DOT__decode_branch_type = 0U;
	    } else {
		if ((8U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		    if ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			vlTOPp->Vortex__DOT__decode_branch_type = 0U;
		    } else {
			if ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			    if ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
				if (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs) 
				     | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt))) {
				    vlTOPp->Vortex__DOT__decode_branch_type = 0U;
				}
			    } else {
				vlTOPp->Vortex__DOT__decode_branch_type = 0U;
			    }
			} else {
			    vlTOPp->Vortex__DOT__decode_branch_type = 0U;
			}
		    }
		} else {
		    vlTOPp->Vortex__DOT__decode_branch_type 
			= ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
			    ? 0U : ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
				     ? ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
					 ? ((0x4000U 
					     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
					     ? ((0x2000U 
						 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						 ? 
						((0x1000U 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						  ? 6U
						  : 5U)
						 : 
						((0x1000U 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						  ? 4U
						  : 3U))
					     : ((0x2000U 
						 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						 ? 0U
						 : 
						((0x1000U 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						  ? 2U
						  : 1U)))
					 : 0U) : 0U));
		}
	    }
	} else {
	    vlTOPp->Vortex__DOT__decode_branch_type = 0U;
	}
    } else {
	vlTOPp->Vortex__DOT__decode_branch_type = 0U;
    }
    vlTOPp->Vortex__DOT__decode_change_mask = ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs) 
					       | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt));
    vlTOPp->Vortex__DOT__e_m_b_reg_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__e_m_b_reg_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__e_m_b_reg_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__e_m_b_reg_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__e_m_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid
	[3U];
    vlTOPp->Vortex__DOT__e_m_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid
	[2U];
    vlTOPp->Vortex__DOT__e_m_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid
	[1U];
    vlTOPp->Vortex__DOT__e_m_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid
	[0U];
    vlTOPp->Vortex__DOT__e_m_alu_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result
	[3U];
    vlTOPp->Vortex__DOT__e_m_alu_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result
	[2U];
    vlTOPp->Vortex__DOT__e_m_alu_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result
	[1U];
    vlTOPp->Vortex__DOT__e_m_alu_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result
	[0U];
    vlTOPp->Vortex__DOT__d_e_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid
	[3U];
    vlTOPp->Vortex__DOT__d_e_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid
	[2U];
    vlTOPp->Vortex__DOT__d_e_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid
	[1U];
    vlTOPp->Vortex__DOT__d_e_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid
	[0U];
    vlTOPp->Vortex__DOT__d_e_a_reg_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__d_e_a_reg_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__d_e_a_reg_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__d_e_a_reg_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__d_e_b_reg_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__d_e_b_reg_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__d_e_b_reg_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__d_e_b_reg_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd 
	= ((((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0xfU)) == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__rd)) 
	      & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				 >> 0xfU)))) & (0U 
						!= (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))) 
	    & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd))) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num) 
	      == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd 
	= ((((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0x14U)) == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__rd)) 
	      & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				 >> 0x14U)))) & (0U 
						 != (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))) 
	    & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd))) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num) 
	      == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__memory_mem_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result
	[3U];
    vlTOPp->Vortex__DOT__memory_mem_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result
	[2U];
    vlTOPp->Vortex__DOT__memory_mem_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result
	[1U];
    vlTOPp->Vortex__DOT__memory_mem_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid[3U] 
	= vlTOPp->Vortex__DOT__m_w_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid[2U] 
	= vlTOPp->Vortex__DOT__m_w_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid[1U] 
	= vlTOPp->Vortex__DOT__m_w_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid[0U] 
	= vlTOPp->Vortex__DOT__m_w_valid[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid[3U] 
	= vlTOPp->Vortex__DOT__f_d_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid[2U] 
	= vlTOPp->Vortex__DOT__f_d_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid[1U] 
	= vlTOPp->Vortex__DOT__f_d_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid[0U] 
	= vlTOPp->Vortex__DOT__f_d_valid[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result[3U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result[2U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result[1U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result[0U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result[3U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result[2U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result[1U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result[0U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result[3U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result[2U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result[1U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result[0U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data[3U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data[2U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data[1U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data[0U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[0U] 
	= (((0U == (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall)) 
	    | (1U != (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall))) 
	   & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[1U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[2U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[3U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[4U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[5U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[6U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[7U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_final_alu 
	= ((0x63U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
	    ? ((5U > (IData)(vlTOPp->Vortex__DOT__decode_branch_type))
	        ? 1U : 0xaU) : ((0x37U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
				 ? 0xbU : ((0x17U == 
					    (0x7fU 
					     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
					    ? 0xcU : 
					   ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_csr)
					     ? ((1U 
						 == 
						 (3U 
						  & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						     >> 0xcU)))
						 ? 0xdU
						 : 
						((2U 
						  == 
						  (3U 
						   & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						      >> 0xcU)))
						  ? 0xeU
						  : 0xfU))
					     : (((0x23U 
						  == 
						  (0x7fU 
						   & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
						 | (3U 
						    == 
						    (0x7fU 
						     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)))
						 ? 0U
						 : 
						((0x4000U 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						  ? 
						 ((0x2000U 
						   & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						   ? 
						  ((0x1000U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 9U
						    : 8U)
						   : 
						  ((0x1000U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 
						   ((0U 
						     == 
						     (0x7fU 
						      & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							 >> 0x19U)))
						     ? 6U
						     : 7U)
						    : 5U))
						  : 
						 ((0x2000U 
						   & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						   ? 
						  ((0x1000U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 4U
						    : 3U)
						   : 
						  ((0x1000U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 2U
						    : 
						   ((0x13U 
						     == 
						     (0x7fU 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
						     ? 0U
						     : 
						    ((0U 
						      == 
						      (0x7fU 
						       & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							  >> 0x19U)))
						      ? 0U
						      : 1U))))))))));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (1U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (2U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (3U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (4U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (5U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (6U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (7U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2[3U] 
	= vlTOPp->Vortex__DOT__e_m_b_reg_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2[2U] 
	= vlTOPp->Vortex__DOT__e_m_b_reg_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2[1U] 
	= vlTOPp->Vortex__DOT__e_m_b_reg_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2[0U] 
	= vlTOPp->Vortex__DOT__e_m_b_reg_data[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid[3U] 
	= vlTOPp->Vortex__DOT__e_m_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid[2U] 
	= vlTOPp->Vortex__DOT__e_m_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid[1U] 
	= vlTOPp->Vortex__DOT__e_m_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid[0U] 
	= vlTOPp->Vortex__DOT__e_m_valid[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result[3U] 
	= vlTOPp->Vortex__DOT__e_m_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result[2U] 
	= vlTOPp->Vortex__DOT__e_m_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result[1U] 
	= vlTOPp->Vortex__DOT__e_m_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result[0U] 
	= vlTOPp->Vortex__DOT__e_m_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid[3U] 
	= vlTOPp->Vortex__DOT__d_e_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid[2U] 
	= vlTOPp->Vortex__DOT__d_e_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid[1U] 
	= vlTOPp->Vortex__DOT__d_e_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid[0U] 
	= vlTOPp->Vortex__DOT__d_e_valid[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data[3U] 
	= vlTOPp->Vortex__DOT__d_e_a_reg_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data[2U] 
	= vlTOPp->Vortex__DOT__d_e_a_reg_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data[1U] 
	= vlTOPp->Vortex__DOT__d_e_a_reg_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data[0U] 
	= vlTOPp->Vortex__DOT__d_e_a_reg_data[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT__d_e_b_reg_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT__d_e_b_reg_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT__d_e_b_reg_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT__d_e_b_reg_data[0U];
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd 
	= (((((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			  >> 0xfU)) == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd)) 
	       & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				  >> 0xfU)))) & (0U 
						 != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))) 
	     & ((IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__warp_num) 
		== (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) 
	    & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd))) 
	   & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)));
    vlTOPp->Vortex__DOT__forwarding_fwd_stall = ((((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd) 
						   | (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd)) 
						  & (2U 
						     == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))) 
						 | (((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd) 
						     | (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)) 
						    & (2U 
						       == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))));
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd 
	= (((((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			  >> 0x14U)) == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd)) 
	       & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				  >> 0x14U)))) & (0U 
						  != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))) 
	     & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd))) 
	    & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd))) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__warp_num) 
	      == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result[3U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result[2U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result[1U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result[0U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data[3U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data[2U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data[1U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data[0U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
	[0U];
    // ALWAYS at VX_decode.v:451
    if ((0x40U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	if ((0x20U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	    if ((0x10U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		vlTOPp->Vortex__DOT__decode_jal = (
						   (~ 
						    (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						     >> 3U)) 
						   & ((~ 
						       (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							>> 2U)) 
						      & ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							  >> 1U) 
							 & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							    & ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__jal_sys_jal) 
							       & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
							       [0U])))));
	    } else {
		if ((8U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		    if ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			vlTOPp->Vortex__DOT__decode_jal 
			    = ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				>> 1U) & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					  & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
					  [0U]));
		    } else {
			if ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			    if ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
				if (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs) 
				     | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt))) {
				    vlTOPp->Vortex__DOT__decode_jal 
					= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
					[0U];
				}
			    } else {
				vlTOPp->Vortex__DOT__decode_jal = 0U;
			    }
			} else {
			    vlTOPp->Vortex__DOT__decode_jal = 0U;
			}
		    }
		} else {
		    vlTOPp->Vortex__DOT__decode_jal 
			= ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			    >> 2U) & ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				       >> 1U) & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						 & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
						 [0U])));
		}
	    }
	} else {
	    vlTOPp->Vortex__DOT__decode_jal = 0U;
	}
    } else {
	vlTOPp->Vortex__DOT__decode_jal = 0U;
    }
    // ALWAYS at VX_decode.v:519
    if ((0x40U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	if ((0x20U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	    if ((0x10U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		vlTOPp->Vortex__DOT__decode_branch_stall = 0U;
	    } else {
		if ((8U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		    if ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			vlTOPp->Vortex__DOT__decode_branch_stall 
			    = ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				>> 1U) & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					  & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
					  [0U]));
		    } else {
			if ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			    if ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
				if (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs) 
				     | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt))) {
				    vlTOPp->Vortex__DOT__decode_branch_stall 
					= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
					[0U];
				}
			    } else {
				vlTOPp->Vortex__DOT__decode_branch_stall = 0U;
			    }
			} else {
			    vlTOPp->Vortex__DOT__decode_branch_stall = 0U;
			}
		    }
		} else {
		    vlTOPp->Vortex__DOT__decode_branch_stall 
			= ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
			    ? ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				>> 1U) & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					  & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
					  [0U])) : 
			   ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			     >> 1U) & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				       & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
				       [0U])));
		}
	    }
	} else {
	    vlTOPp->Vortex__DOT__decode_branch_stall = 0U;
	}
    } else {
	vlTOPp->Vortex__DOT__decode_branch_stall = 0U;
    }
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_ebreak 
	= ((0x73U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__jal_sys_jal) 
	      & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
	      [0U]));
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data[3U] 
	= ((3U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
	    ? vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data
	   [3U] : ((1U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
		    ? vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result
		   [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result
		   [3U]));
    vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data[2U] 
	= ((3U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
	    ? vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data
	   [2U] : ((1U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
		    ? vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result
		   [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result
		   [2U]));
    vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data[1U] 
	= ((3U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
	    ? vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data
	   [1U] : ((1U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
		    ? vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result
		   [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result
		   [1U]));
    vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data[0U] 
	= ((3U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
	    ? vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data
	   [0U] : ((1U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
		    ? vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result
		   [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result
		   [0U]));
    // ALWAYS at VX_decode.v:276
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [1U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [2U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [3U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [4U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [5U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [6U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [7U]);
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[0U];
    // ALWAYS at VX_memory.v:113
    vlTOPp->Vortex__DOT__memory_branch_dir = (1U & 
					      ((4U 
						& (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type))
					        ? (
						   (2U 
						    & (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type))
						    ? 
						   ((~ (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type)) 
						    & (~ 
						       (vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
							[0U] 
							>> 0x1fU)))
						    : 
						   ((1U 
						     & (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type))
						     ? 
						    (vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
						     [0U] 
						     >> 0x1fU)
						     : 
						    (~ 
						     (vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
						      [0U] 
						      >> 0x1fU))))
					        : (
						   (2U 
						    & (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type))
						    ? 
						   ((1U 
						     & (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type))
						     ? 
						    (vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
						     [0U] 
						     >> 0x1fU)
						     : 
						    (0U 
						     != 
						     vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
						     [0U]))
						    : 
						   ((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type) 
						    & (0U 
						       == 
						       vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
						       [0U])))));
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rs2_src)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed
	    : vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	   [0U]);
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rs2_src)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed
	    : vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	   [1U]);
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rs2_src)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed
	    : vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	   [2U]);
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rs2_src)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed
	    : vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	   [3U]);
    vlTOPp->Vortex__DOT__forwarding_src1_fwd = (((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd) 
						 | (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)) 
						| (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd));
    vlTOPp->Vortex__DOT__forwarding_src2_fwd = (((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd) 
						 | (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)) 
						| (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd));
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlTOPp->Vortex__DOT__decode_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid
	[3U];
    vlTOPp->Vortex__DOT__decode_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid
	[2U];
    vlTOPp->Vortex__DOT__decode_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid
	[1U];
    vlTOPp->Vortex__DOT__decode_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid
	[0U];
    vlTOPp->out_ebreak = ((0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)) 
			  & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_ebreak));
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__writeback_write_data[3U] = 
	vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data
	[3U];
    vlTOPp->Vortex__DOT__writeback_write_data[2U] = 
	vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data
	[2U];
    vlTOPp->Vortex__DOT__writeback_write_data[1U] = 
	vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data
	[1U];
    vlTOPp->Vortex__DOT__writeback_write_data[0U] = 
	vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__add_warp = 
	(((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	  & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_ebreak))) 
	 & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall)));
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling 
	= (((IData)(vlTOPp->Vortex__DOT__forwarding_fwd_stall) 
	    | (IData)(vlTOPp->Vortex__DOT__execute_branch_stall)) 
	   | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__stall = ((((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
						   | (IData)(vlTOPp->Vortex__DOT__decode_branch_stall)) 
						  | (IData)(vlTOPp->Vortex__DOT__forwarding_fwd_stall)) 
						 | (IData)(vlTOPp->Vortex__DOT__execute_branch_stall));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp 
	= (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_ebreak) 
	    & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn))) 
	   & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall)));
    vlTOPp->out_cache_driver_in_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data
	[3U];
    vlTOPp->out_cache_driver_in_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data
	[2U];
    vlTOPp->out_cache_driver_in_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data
	[1U];
    vlTOPp->out_cache_driver_in_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data
	[0U];
    vlTOPp->out_cache_driver_in_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid
	[3U];
    vlTOPp->out_cache_driver_in_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid
	[2U];
    vlTOPp->out_cache_driver_in_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid
	[1U];
    vlTOPp->out_cache_driver_in_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid
	[0U];
    vlTOPp->Vortex__DOT__memory_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid
	[3U];
    vlTOPp->Vortex__DOT__memory_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid
	[2U];
    vlTOPp->Vortex__DOT__memory_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid
	[1U];
    vlTOPp->Vortex__DOT__memory_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid
	[0U];
    vlTOPp->out_cache_driver_in_address[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address
	[3U];
    vlTOPp->out_cache_driver_in_address[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address
	[2U];
    vlTOPp->out_cache_driver_in_address[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address
	[1U];
    vlTOPp->out_cache_driver_in_address[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address
	[0U];
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (0U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (0U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (1U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (1U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (4U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (4U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (5U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (5U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (6U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (6U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (7U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (7U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__real_PC));
    vlTOPp->Vortex__DOT__memory_alu_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result
	[3U];
    vlTOPp->Vortex__DOT__memory_alu_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result
	[2U];
    vlTOPp->Vortex__DOT__memory_alu_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result
	[1U];
    vlTOPp->Vortex__DOT__memory_alu_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result
	[0U];
    vlTOPp->Vortex__DOT__execute_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid
	[3U];
    vlTOPp->Vortex__DOT__execute_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid
	[2U];
    vlTOPp->Vortex__DOT__execute_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid
	[1U];
    vlTOPp->Vortex__DOT__execute_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid
	[0U];
    vlTOPp->Vortex__DOT__execute_b_reg_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__execute_b_reg_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__execute_b_reg_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__execute_b_reg_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__mult_signed_result 
	= VL_MULS_QQQ(64,64,64, VL_EXTENDS_QI(64,32, 
					      vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					      [0U]), 
		      VL_EXTENDS_QI(64,32, vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2));
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__mult_signed_result 
	= VL_MULS_QQQ(64,64,64, VL_EXTENDS_QI(64,32, 
					      vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					      [1U]), 
		      VL_EXTENDS_QI(64,32, vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2));
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__mult_signed_result 
	= VL_MULS_QQQ(64,64,64, VL_EXTENDS_QI(64,32, 
					      vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					      [2U]), 
		      VL_EXTENDS_QI(64,32, vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2));
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__mult_signed_result 
	= VL_MULS_QQQ(64,64,64, VL_EXTENDS_QI(64,32, 
					      vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					      [3U]), 
		      VL_EXTENDS_QI(64,32, vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2));
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid[3U] 
	= vlTOPp->Vortex__DOT__decode_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid[2U] 
	= vlTOPp->Vortex__DOT__decode_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid[1U] 
	= vlTOPp->Vortex__DOT__decode_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid[0U] 
	= vlTOPp->Vortex__DOT__decode_valid[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data[3U] 
	= vlTOPp->Vortex__DOT__writeback_write_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data[2U] 
	= vlTOPp->Vortex__DOT__writeback_write_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data[1U] 
	= vlTOPp->Vortex__DOT__writeback_write_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data[0U] 
	= vlTOPp->Vortex__DOT__writeback_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (0U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (1U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (2U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (3U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (4U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (5U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (6U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (7U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid[3U] 
	= vlTOPp->Vortex__DOT__memory_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid[2U] 
	= vlTOPp->Vortex__DOT__memory_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid[1U] 
	= vlTOPp->Vortex__DOT__memory_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid[0U] 
	= vlTOPp->Vortex__DOT__memory_valid[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[4U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[5U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[6U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[7U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result[3U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result[2U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result[1U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result[0U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result[3U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result[2U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result[1U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result[0U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid[3U] 
	= vlTOPp->Vortex__DOT__execute_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid[2U] 
	= vlTOPp->Vortex__DOT__execute_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid[1U] 
	= vlTOPp->Vortex__DOT__execute_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid[0U] 
	= vlTOPp->Vortex__DOT__execute_valid[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT__execute_b_reg_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT__execute_b_reg_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT__execute_b_reg_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT__execute_b_reg_data[0U];
    // ALWAYS at VX_alu.v:48
    vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__0__KET____DOT__vx_alu__out_alu_result 
	= ((0x10U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	    ? ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? 0U : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			 ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [0U] : VL_MODDIV_III(32, 
							 vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							 [0U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [0U] : VL_MODDIVS_III(32, 
							  vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							  [0U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIV_III(32, 
					       vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					       [0U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIVS_III(32, 
						vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						[0U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))))
			 : ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((((QData)((IData)(
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [0U])) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U))
				 : (IData)((((((QData)((IData)(
							       VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
										[0U] 
										>> 0x1fU)))))) 
					       << 0x20U) 
					      | (QData)((IData)(
								vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
								[0U]))) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__mult_signed_result 
					    >> 0x20U))
				 : (IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__mult_signed_result)))))
	    : ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? vlTOPp->Vortex__DOT__csr_decode_csr_data
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? vlTOPp->Vortex__DOT__csr_decode_csr_data
			    : (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC 
			       + (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
				  << 0xcU)))) : ((2U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
						   << 0xcU)
						   : 
						  ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						    [0U] 
						    >= vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
						    ? 0U
						    : 0xffffffffU))
						  : 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2 
						   & vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [0U])
						   : 
						  (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [0U] 
						   | vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))))
	        : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? VL_SHIFTRS_III(32,32,5, 
					     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					     [0U], 
					     (0x1fU 
					      & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))
			    : (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [0U] >> (0x1fU & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)))
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [0U] ^ vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
			    : ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				[0U] < vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
			        ? 1U : 0U))) : ((2U 
						 & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						 ? 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (VL_LTS_III(1,32,32, 
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [0U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
						   ? 1U
						   : 0U)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [0U] 
						  << 
						  (0x1fU 
						   & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)))
						 : 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [0U] 
						  - vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [0U] 
						  + vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))))));
    // ALWAYS at VX_alu.v:48
    vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__1__KET____DOT__vx_alu__out_alu_result 
	= ((0x10U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	    ? ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? 0U : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			 ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [1U] : VL_MODDIV_III(32, 
							 vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							 [1U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [1U] : VL_MODDIVS_III(32, 
							  vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							  [1U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIV_III(32, 
					       vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					       [1U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIVS_III(32, 
						vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						[1U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))))
			 : ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((((QData)((IData)(
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [1U])) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U))
				 : (IData)((((((QData)((IData)(
							       VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
										[1U] 
										>> 0x1fU)))))) 
					       << 0x20U) 
					      | (QData)((IData)(
								vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
								[1U]))) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__mult_signed_result 
					    >> 0x20U))
				 : (IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__mult_signed_result)))))
	    : ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? vlTOPp->Vortex__DOT__csr_decode_csr_data
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? vlTOPp->Vortex__DOT__csr_decode_csr_data
			    : (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC 
			       + (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
				  << 0xcU)))) : ((2U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
						   << 0xcU)
						   : 
						  ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						    [1U] 
						    >= vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
						    ? 0U
						    : 0xffffffffU))
						  : 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2 
						   & vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [1U])
						   : 
						  (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [1U] 
						   | vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))))
	        : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? VL_SHIFTRS_III(32,32,5, 
					     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					     [1U], 
					     (0x1fU 
					      & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))
			    : (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [1U] >> (0x1fU & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)))
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [1U] ^ vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
			    : ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				[1U] < vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
			        ? 1U : 0U))) : ((2U 
						 & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						 ? 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (VL_LTS_III(1,32,32, 
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [1U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
						   ? 1U
						   : 0U)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [1U] 
						  << 
						  (0x1fU 
						   & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)))
						 : 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [1U] 
						  - vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [1U] 
						  + vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))))));
    // ALWAYS at VX_alu.v:48
    vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__2__KET____DOT__vx_alu__out_alu_result 
	= ((0x10U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	    ? ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? 0U : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			 ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [2U] : VL_MODDIV_III(32, 
							 vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							 [2U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [2U] : VL_MODDIVS_III(32, 
							  vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							  [2U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIV_III(32, 
					       vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					       [2U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIVS_III(32, 
						vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						[2U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))))
			 : ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((((QData)((IData)(
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [2U])) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U))
				 : (IData)((((((QData)((IData)(
							       VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
										[2U] 
										>> 0x1fU)))))) 
					       << 0x20U) 
					      | (QData)((IData)(
								vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
								[2U]))) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__mult_signed_result 
					    >> 0x20U))
				 : (IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__mult_signed_result)))))
	    : ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? vlTOPp->Vortex__DOT__csr_decode_csr_data
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? vlTOPp->Vortex__DOT__csr_decode_csr_data
			    : (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC 
			       + (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
				  << 0xcU)))) : ((2U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
						   << 0xcU)
						   : 
						  ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						    [2U] 
						    >= vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
						    ? 0U
						    : 0xffffffffU))
						  : 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2 
						   & vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [2U])
						   : 
						  (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [2U] 
						   | vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))))
	        : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? VL_SHIFTRS_III(32,32,5, 
					     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					     [2U], 
					     (0x1fU 
					      & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))
			    : (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [2U] >> (0x1fU & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)))
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [2U] ^ vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
			    : ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				[2U] < vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
			        ? 1U : 0U))) : ((2U 
						 & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						 ? 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (VL_LTS_III(1,32,32, 
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [2U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
						   ? 1U
						   : 0U)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [2U] 
						  << 
						  (0x1fU 
						   & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)))
						 : 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [2U] 
						  - vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [2U] 
						  + vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))))));
    // ALWAYS at VX_alu.v:48
    vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__3__KET____DOT__vx_alu__out_alu_result 
	= ((0x10U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	    ? ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? 0U : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			 ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [3U] : VL_MODDIV_III(32, 
							 vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							 [3U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [3U] : VL_MODDIVS_III(32, 
							  vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							  [3U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIV_III(32, 
					       vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					       [3U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIVS_III(32, 
						vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						[3U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))))
			 : ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((((QData)((IData)(
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [3U])) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U))
				 : (IData)((((((QData)((IData)(
							       VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
										[3U] 
										>> 0x1fU)))))) 
					       << 0x20U) 
					      | (QData)((IData)(
								vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
								[3U]))) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__mult_signed_result 
					    >> 0x20U))
				 : (IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__mult_signed_result)))))
	    : ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? vlTOPp->Vortex__DOT__csr_decode_csr_data
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? vlTOPp->Vortex__DOT__csr_decode_csr_data
			    : (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC 
			       + (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
				  << 0xcU)))) : ((2U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
						   << 0xcU)
						   : 
						  ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						    [3U] 
						    >= vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
						    ? 0U
						    : 0xffffffffU))
						  : 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2 
						   & vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [3U])
						   : 
						  (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [3U] 
						   | vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))))
	        : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? VL_SHIFTRS_III(32,32,5, 
					     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					     [3U], 
					     (0x1fU 
					      & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))
			    : (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [3U] >> (0x1fU & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)))
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [3U] ^ vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
			    : ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				[3U] < vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
			        ? 1U : 0U))) : ((2U 
						 & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						 ? 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (VL_LTS_III(1,32,32, 
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [3U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
						   ? 1U
						   : 0U)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [3U] 
						  << 
						  (0x1fU 
						   & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)))
						 : 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [3U] 
						  - vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [3U] 
						  + vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))))));
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    // ALWAYS at VX_fetch.v:172
    if ((0U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [0U];
    }
    if ((1U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [1U];
    }
    if ((2U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [2U];
    }
    if ((3U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [3U];
    }
    if ((4U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [4U];
    }
    if ((5U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [5U];
    }
    if ((6U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [6U];
    }
    if ((7U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [7U];
    }
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result[0U] 
	= vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__0__KET____DOT__vx_alu__out_alu_result;
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result[1U] 
	= vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__1__KET____DOT__vx_alu__out_alu_result;
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result[2U] 
	= vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__2__KET____DOT__vx_alu__out_alu_result;
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result[3U] 
	= vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__3__KET____DOT__vx_alu__out_alu_result;
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlTOPp->icache_request_pc_address = vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var;
    vlTOPp->Vortex__DOT__execute_alu_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result
	[3U];
    vlTOPp->Vortex__DOT__execute_alu_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result
	[2U];
    vlTOPp->Vortex__DOT__execute_alu_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result
	[1U];
    vlTOPp->Vortex__DOT__execute_alu_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result[3U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result[2U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result[1U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result[0U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result[3U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result[2U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result[1U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result[0U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [3U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [3U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [3U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [3U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [3U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [3U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [3U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [2U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [2U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [2U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [2U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [2U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [2U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [2U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [1U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [1U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [1U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [1U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [1U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [1U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [1U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [0U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [0U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [0U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [0U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [0U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [0U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [0U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [3U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [3U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [3U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [3U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [3U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [3U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [3U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [2U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [2U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [2U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [2U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [2U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [2U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [2U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [1U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [1U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [1U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [1U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [1U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [1U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [1U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [0U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [0U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [0U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [0U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [0U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [0U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [0U])));
    vlTOPp->Vortex__DOT__forwarding_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__forwarding_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__forwarding_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__forwarding_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__forwarding_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__forwarding_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__forwarding_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__forwarding_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__forwarding_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__forwarding_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__forwarding_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__forwarding_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__forwarding_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__forwarding_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__forwarding_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__forwarding_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data[0U] 
	= ((0x6fU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
	    ? vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC
	    : ((IData)(vlTOPp->Vortex__DOT__forwarding_src1_fwd)
	        ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data
	       [0U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
	       [0U]));
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data[1U] 
	= ((0x6fU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
	    ? vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC
	    : ((IData)(vlTOPp->Vortex__DOT__forwarding_src1_fwd)
	        ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data
	       [1U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
	       [1U]));
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data[2U] 
	= ((0x6fU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
	    ? vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC
	    : ((IData)(vlTOPp->Vortex__DOT__forwarding_src1_fwd)
	        ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data
	       [2U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
	       [2U]));
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data[3U] 
	= ((0x6fU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
	    ? vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC
	    : ((IData)(vlTOPp->Vortex__DOT__forwarding_src1_fwd)
	        ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data
	       [3U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
	       [3U]));
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__forwarding_src2_fwd)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data
	   [0U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register
	   [0U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__forwarding_src2_fwd)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data
	   [1U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register
	   [1U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__forwarding_src2_fwd)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data
	   [2U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register
	   [2U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__forwarding_src2_fwd)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data
	   [3U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register
	   [3U]);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[0U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[0U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[0U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[0U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[0U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[0U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[0U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[0U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data
	[0U];
}

VL_INLINE_OPT void VVortex::_settle__TOP__3(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_settle__TOP__3\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_a_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_a_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_a_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_a_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_b_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_b_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_b_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_b_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_a_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_a_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_a_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_a_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_b_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_b_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_b_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_b_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_a_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_a_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_a_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_a_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_b_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_b_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_b_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_b_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_a_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_a_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_a_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_a_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_b_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_b_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_b_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_b_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_a_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_a_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_a_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_a_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_b_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_b_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_b_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_b_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_a_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_a_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_a_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_a_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_b_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_b_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_b_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_b_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_a_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_a_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_a_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_a_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_b_reg_data[3U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_b_reg_data[2U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_b_reg_data[1U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_b_reg_data[0U] 
	= vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[1U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[1U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[1U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[1U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[1U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[1U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[1U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[1U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[2U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[2U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[2U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[2U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[2U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[2U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[2U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[2U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[3U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[3U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[3U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[3U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[3U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[3U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[3U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[3U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[4U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[4U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[4U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[4U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[4U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[4U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[4U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[4U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[5U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[5U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[5U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[5U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[5U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[5U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[5U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[5U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[6U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[6U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[6U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[6U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[6U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[6U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[6U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[6U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[7U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[7U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[7U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[7U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[7U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[7U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[7U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[7U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_b_reg_data
	[0U];
    // ALWAYS at VX_decode.v:247
    if ((0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [0U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [0U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [0U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [0U][0U];
    }
    if ((1U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [1U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [1U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [1U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [1U][0U];
    }
    if ((2U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [2U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [2U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [2U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [2U][0U];
    }
    if ((3U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [3U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [3U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [3U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [3U][0U];
    }
    if ((4U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [4U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [4U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [4U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [4U][0U];
    }
    if ((5U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [5U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [5U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [5U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [5U][0U];
    }
    if ((6U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [6U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [6U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [6U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [6U][0U];
    }
    if ((7U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [7U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [7U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [7U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data
	    [7U][0U];
    }
    // ALWAYS at VX_decode.v:247
    if ((0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [0U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [0U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [0U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [0U][0U];
    }
    if ((1U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [1U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [1U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [1U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [1U][0U];
    }
    if ((2U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [2U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [2U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [2U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [2U][0U];
    }
    if ((3U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [3U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [3U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [3U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [3U][0U];
    }
    if ((4U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [4U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [4U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [4U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [4U][0U];
    }
    if ((5U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [5U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [5U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [5U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [5U][0U];
    }
    if ((6U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [6U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [6U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [6U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [6U][0U];
    }
    if ((7U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[3U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [7U][3U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[2U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [7U][2U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[1U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [7U][1U];
	vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[0U] 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data
	    [7U][0U];
    }
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__decode_a_reg_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__decode_a_reg_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__decode_a_reg_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__decode_a_reg_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__decode_b_reg_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__decode_b_reg_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__decode_b_reg_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__decode_b_reg_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__jalrs_thread_mask[0U] 
	= VL_LTES_III(1,32,32, 0U, vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data
		      [0U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__jalrs_thread_mask[1U] 
	= VL_LTES_III(1,32,32, 1U, vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data
		      [0U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__jalrs_thread_mask[2U] 
	= VL_LTES_III(1,32,32, 2U, vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data
		      [0U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__jalrs_thread_mask[3U] 
	= VL_LTES_III(1,32,32, 3U, vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_b_reg_data
		      [0U]);
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_a_reg_data[3U] 
	= vlTOPp->Vortex__DOT__decode_a_reg_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_a_reg_data[2U] 
	= vlTOPp->Vortex__DOT__decode_a_reg_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_a_reg_data[1U] 
	= vlTOPp->Vortex__DOT__decode_a_reg_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_a_reg_data[0U] 
	= vlTOPp->Vortex__DOT__decode_a_reg_data[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT__decode_b_reg_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT__decode_b_reg_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT__decode_b_reg_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT__decode_b_reg_data[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_thread_mask[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT__jalrs_thread_mask
	   [3U] : vlTOPp->Vortex__DOT__vx_decode__DOT__jmprt_thread_mask
	   [3U]);
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_thread_mask[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT__jalrs_thread_mask
	   [2U] : vlTOPp->Vortex__DOT__vx_decode__DOT__jmprt_thread_mask
	   [2U]);
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_thread_mask[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT__jalrs_thread_mask
	   [1U] : vlTOPp->Vortex__DOT__vx_decode__DOT__jmprt_thread_mask
	   [1U]);
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_thread_mask[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT__jalrs_thread_mask
	   [0U] : vlTOPp->Vortex__DOT__vx_decode__DOT__jmprt_thread_mask
	   [0U]);
    vlTOPp->Vortex__DOT__decode_thread_mask[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_thread_mask
	[3U];
    vlTOPp->Vortex__DOT__decode_thread_mask[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_thread_mask
	[2U];
    vlTOPp->Vortex__DOT__decode_thread_mask[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_thread_mask
	[1U];
    vlTOPp->Vortex__DOT__decode_thread_mask[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_thread_mask
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask[3U] 
	= vlTOPp->Vortex__DOT__decode_thread_mask[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask[2U] 
	= vlTOPp->Vortex__DOT__decode_thread_mask[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask[1U] 
	= vlTOPp->Vortex__DOT__decode_thread_mask[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask[0U] 
	= vlTOPp->Vortex__DOT__decode_thread_mask[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask
	[0U];
    // ALWAYS at VX_warp.v:41
    if (((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp) 
	 & (0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid_zero
	    [3U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid_zero
	    [2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid_zero
	    [1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid_zero
	    [0U];
    } else {
	if (vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_change_mask) {
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[3U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask
		[3U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[2U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask
		[2U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[1U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask
		[1U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[0U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask
		[0U];
	}
    }
    // ALWAYS at VX_warp.v:41
    if (((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp) 
	 & (1U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid_zero
	    [3U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid_zero
	    [2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid_zero
	    [1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid_zero
	    [0U];
    } else {
	if (vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_change_mask) {
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[3U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask
		[3U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[2U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask
		[2U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[1U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask
		[1U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[0U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask
		[0U];
	}
    }
    // ALWAYS at VX_warp.v:41
    if (((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp) 
	 & (2U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid_zero
	    [3U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid_zero
	    [2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid_zero
	    [1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid_zero
	    [0U];
    } else {
	if (vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_change_mask) {
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[3U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask
		[3U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[2U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask
		[2U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[1U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask
		[1U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[0U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask
		[0U];
	}
    }
    // ALWAYS at VX_warp.v:41
    if (((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp) 
	 & (3U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid_zero
	    [3U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid_zero
	    [2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid_zero
	    [1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid_zero
	    [0U];
    } else {
	if (vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_change_mask) {
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[3U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask
		[3U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[2U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask
		[2U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[1U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask
		[1U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[0U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask
		[0U];
	}
    }
    // ALWAYS at VX_warp.v:41
    if (((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp) 
	 & (4U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid_zero
	    [3U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid_zero
	    [2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid_zero
	    [1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid_zero
	    [0U];
    } else {
	if (vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_change_mask) {
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[3U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask
		[3U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[2U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask
		[2U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[1U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask
		[1U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[0U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask
		[0U];
	}
    }
    // ALWAYS at VX_warp.v:41
    if (((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp) 
	 & (5U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid_zero
	    [3U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid_zero
	    [2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid_zero
	    [1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid_zero
	    [0U];
    } else {
	if (vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_change_mask) {
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[3U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask
		[3U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[2U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask
		[2U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[1U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask
		[1U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[0U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask
		[0U];
	}
    }
    // ALWAYS at VX_warp.v:41
    if (((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp) 
	 & (6U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid_zero
	    [3U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid_zero
	    [2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid_zero
	    [1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid_zero
	    [0U];
    } else {
	if (vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_change_mask) {
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[3U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask
		[3U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[2U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask
		[2U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[1U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask
		[1U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[0U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask
		[0U];
	}
    }
    // ALWAYS at VX_warp.v:41
    if (((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp) 
	 & (7U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid_zero
	    [3U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid_zero
	    [2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid_zero
	    [1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid_zero
	    [0U];
    } else {
	if (vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_change_mask) {
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[3U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask
		[3U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[2U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask
		[2U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[1U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask
		[1U];
	    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[0U] 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask
		[0U];
	}
    }
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__0__KET____DOT__VX_Warp__out_valid[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask
	   [0U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid
		   [0U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__0__KET____DOT__VX_Warp__out_valid[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask
	   [1U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid
		   [1U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__0__KET____DOT__VX_Warp__out_valid[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask
	   [2U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid
		   [2U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__0__KET____DOT__VX_Warp__out_valid[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask
	   [3U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid
		   [3U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Warp__out_valid[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask
	   [0U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid
		   [0U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Warp__out_valid[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask
	   [1U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid
		   [1U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Warp__out_valid[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask
	   [2U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid
		   [2U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Warp__out_valid[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask
	   [3U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid
		   [3U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Warp__out_valid[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask
	   [0U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid
		   [0U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Warp__out_valid[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask
	   [1U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid
		   [1U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Warp__out_valid[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask
	   [2U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid
		   [2U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Warp__out_valid[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask
	   [3U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid
		   [3U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Warp__out_valid[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask
	   [0U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid
		   [0U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Warp__out_valid[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask
	   [1U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid
		   [1U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Warp__out_valid[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask
	   [2U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid
		   [2U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Warp__out_valid[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask
	   [3U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid
		   [3U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Warp__out_valid[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask
	   [0U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid
		   [0U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Warp__out_valid[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask
	   [1U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid
		   [1U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Warp__out_valid[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask
	   [2U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid
		   [2U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Warp__out_valid[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask
	   [3U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid
		   [3U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Warp__out_valid[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask
	   [0U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid
		   [0U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Warp__out_valid[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask
	   [1U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid
		   [1U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Warp__out_valid[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask
	   [2U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid
		   [2U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Warp__out_valid[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask
	   [3U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid
		   [3U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Warp__out_valid[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask
	   [0U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid
		   [0U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Warp__out_valid[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask
	   [1U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid
		   [1U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Warp__out_valid[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask
	   [2U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid
		   [2U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Warp__out_valid[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask
	   [3U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid
		   [3U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Warp__out_valid[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask
	   [0U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid
		   [0U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Warp__out_valid[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask
	   [1U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid
		   [1U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Warp__out_valid[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask
	   [2U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid
		   [2U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Warp__out_valid[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_change_mask)
	    ? vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask
	   [3U] : ((~ (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_stall)) 
		   & vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid
		   [3U]));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[0U][3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__0__KET____DOT__VX_Warp__out_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[0U][2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__0__KET____DOT__VX_Warp__out_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[0U][1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__0__KET____DOT__VX_Warp__out_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[0U][0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__0__KET____DOT__VX_Warp__out_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[1U][3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Warp__out_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[1U][2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Warp__out_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[1U][1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Warp__out_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[1U][0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Warp__out_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[2U][3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Warp__out_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[2U][2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Warp__out_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[2U][1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Warp__out_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[2U][0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Warp__out_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[3U][3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Warp__out_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[3U][2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Warp__out_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[3U][1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Warp__out_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[3U][0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Warp__out_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[4U][3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Warp__out_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[4U][2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Warp__out_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[4U][1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Warp__out_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[4U][0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Warp__out_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[5U][3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Warp__out_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[5U][2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Warp__out_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[5U][1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Warp__out_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[5U][0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Warp__out_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[6U][3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Warp__out_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[6U][2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Warp__out_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[6U][1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Warp__out_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[6U][0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Warp__out_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[7U][3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Warp__out_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[7U][2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Warp__out_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[7U][1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Warp__out_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid[7U][0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Warp__out_valid
	[0U];
    // ALWAYS at VX_fetch.v:172
    if ((0U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [0U][0U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [0U][1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [0U][2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [0U][3U];
    }
    if ((1U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [1U][0U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [1U][1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [1U][2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [1U][3U];
    }
    if ((2U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [2U][0U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [2U][1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [2U][2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [2U][3U];
    }
    if ((3U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [3U][0U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [3U][1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [3U][2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [3U][3U];
    }
    if ((4U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [4U][0U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [4U][1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [4U][2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [4U][3U];
    }
    if ((5U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [5U][0U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [5U][1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [5U][2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [5U][3U];
    }
    if ((6U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [6U][0U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [6U][1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [6U][2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [6U][3U];
    }
    if ((7U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[0U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [7U][0U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[1U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [7U][1U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[2U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [7U][2U];
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var[3U] 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
	    [7U][3U];
    }
    vlTOPp->Vortex__DOT____Vcellout__vx_fetch__out_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_fetch__out_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_fetch__out_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_fetch__out_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__out_valid_var
	[0U];
    vlTOPp->Vortex__DOT__fetch_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_fetch__out_valid
	[3U];
    vlTOPp->Vortex__DOT__fetch_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_fetch__out_valid
	[2U];
    vlTOPp->Vortex__DOT__fetch_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_fetch__out_valid
	[1U];
    vlTOPp->Vortex__DOT__fetch_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_fetch__out_valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_f_d_reg__in_valid[3U] 
	= vlTOPp->Vortex__DOT__fetch_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_f_d_reg__in_valid[2U] 
	= vlTOPp->Vortex__DOT__fetch_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_f_d_reg__in_valid[1U] 
	= vlTOPp->Vortex__DOT__fetch_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_f_d_reg__in_valid[0U] 
	= vlTOPp->Vortex__DOT__fetch_valid[0U];
}

VL_INLINE_OPT void VVortex::_sequent__TOP__4(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_sequent__TOP__4\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Variables
    // Begin mtask footprint  all: 
    VL_SIG8(__Vdly__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall,5,0);
    VL_SIG8(__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers__v0,4,0);
    VL_SIG8(__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers__v0,0,0);
    VL_SIG8(__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v0,4,0);
    VL_SIG8(__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v0,0,0);
    VL_SIG8(__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v1,0,0);
    VL_SIG8(__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v0,4,0);
    VL_SIG8(__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v0,0,0);
    VL_SIG8(__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v1,0,0);
    VL_SIG8(__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v0,4,0);
    VL_SIG8(__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v0,0,0);
    VL_SIG8(__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v1,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v0,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v1,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v2,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v3,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v0,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v1,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v2,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v3,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v0,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v1,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v2,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v3,0,0);
    VL_SIG8(__Vdlyvset__Vortex__DOT__vx_csr_handler__DOT__csr__v0,0,0);
    VL_SIG16(__Vdlyvdim0__Vortex__DOT__vx_csr_handler__DOT__csr__v0,10,0);
    VL_SIG16(__Vdlyvval__Vortex__DOT__vx_csr_handler__DOT__csr__v0,11,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers__v0,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v0,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v1,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v2,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v3,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v4,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v5,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v6,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v7,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v8,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v9,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v10,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v11,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v12,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v13,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v14,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v15,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v16,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v17,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v18,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v19,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v20,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v21,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v22,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v23,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v24,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v25,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v26,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v27,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v28,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v29,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v30,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v31,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v32,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v0,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v1,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v2,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v3,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v4,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v5,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v6,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v7,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v8,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v9,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v10,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v11,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v12,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v13,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v14,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v15,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v16,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v17,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v18,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v19,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v20,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v21,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v22,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v23,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v24,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v25,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v26,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v27,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v28,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v29,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v30,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v31,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v32,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v0,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v1,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v2,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v3,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v4,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v5,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v6,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v7,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v8,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v9,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v10,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v11,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v12,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v13,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v14,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v15,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v16,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v17,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v18,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v19,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v20,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v21,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v22,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v23,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v24,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v25,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v26,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v27,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v28,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v29,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v30,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v31,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v32,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v0,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v1,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v2,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v3,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v0,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v1,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v2,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v3,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v0,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v1,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v2,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v3,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v0,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v1,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v2,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v3,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v0,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v1,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v2,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v3,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v0,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v1,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v2,31,0);
    VL_SIG(__Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v3,31,0);
    // Body
    __Vdly__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall;
    __Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers__v0 = 0U;
    __Vdlyvset__Vortex__DOT__vx_csr_handler__DOT__csr__v0 = 0U;
    __Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v0 = 0U;
    __Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v1 = 0U;
    __Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v0 = 0U;
    __Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v1 = 0U;
    __Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v0 = 0U;
    __Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v1 = 0U;
    // ALWAYS at VX_e_m_reg.v:128
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v0 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data
	[3U];
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v1 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data
	[2U];
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v2 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data
	[1U];
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v3 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data
	[0U];
    // ALWAYS at VX_m_w_reg.v:63
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v0 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid
	[3U];
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v1 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid
	[2U];
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v2 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid
	[1U];
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v3 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid
	[0U];
    // ALWAYS at VX_e_m_reg.v:128
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v0 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid
	[3U];
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v1 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid
	[2U];
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v2 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid
	[1U];
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v3 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid
	[0U];
    // ALWAYS at VX_context.v:83
    if (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone) 
	 & (0U == (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall)))) {
	__Vdly__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall = 0xaU;
    } else {
	if ((1U == (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall))) {
	    __Vdly__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall = 0U;
	} else {
	    if ((0U < (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall))) {
		__Vdly__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall 
		    = (0x3fU & ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall) 
				- (IData)(1U)));
	    }
	}
    }
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
	= (0xfffffU & ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
		        ? 0U : ((0x37U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
				 ? (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				    >> 0xcU) : ((0x17U 
						 == 
						 (0x7fU 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
						 ? 
						(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						 >> 0xcU)
						 : 0U))));
    // ALWAYS at VX_m_w_reg.v:63
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v0 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result
	[3U];
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v1 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result
	[2U];
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v2 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result
	[1U];
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v3 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result
	[0U];
    // ALWAYS at VX_m_w_reg.v:63
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v0 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result
	[3U];
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v1 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result
	[2U];
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v2 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result
	[1U];
    __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v3 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result
	[0U];
    // ALWAYS at VX_csr_handler.v:36
    vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address 
	= vlTOPp->Vortex__DOT__decode_csr_address;
    // ALWAYS at VX_csr_handler.v:36
    if (vlTOPp->Vortex__DOT__m_w_valid[0U]) {
	vlTOPp->Vortex__DOT__vx_csr_handler__DOT__instret 
	    = (VL_ULL(1) + vlTOPp->Vortex__DOT__vx_csr_handler__DOT__instret);
    }
    // ALWAYS at VX_csr_handler.v:36
    vlTOPp->Vortex__DOT__vx_csr_handler__DOT__cycle 
	= (VL_ULL(1) + vlTOPp->Vortex__DOT__vx_csr_handler__DOT__cycle);
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rs2_src = 
	(1U & ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	        ? 0U : (1U & (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_itype) 
			       | (0x23U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)))
			       ? 1U : 0U))));
    // ALWAYS at VX_e_m_reg.v:128
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v0 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result
	[3U];
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v1 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result
	[2U];
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v2 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result
	[1U];
    __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v3 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result
	[0U];
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__branch_type;
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal = vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__jal;
    // ALWAYS at VX_d_e_reg.v:145
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v0 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid_z
	   [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid
	   [3U]);
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v1 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid_z
	   [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid
	   [2U]);
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid_z
	   [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid
	   [1U]);
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v3 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid_z
	   [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid
	   [0U]);
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__mem_write 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__mem_write;
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__mem_read 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__mem_read;
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__curr_PC = vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC;
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_offset 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed;
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest 
	= (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
	   [0U] + vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__jal_offset);
    // ALWAYS at VX_m_w_reg.v:63
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next = vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next;
    // ALWAYS at VX_register_file.v:45
    if (((((0U != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb)) 
	   & (0U != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd))) 
	  & vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid
	  [0U]) & (0U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__warp_num)))) {
	__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers__v0 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data
	    [0U];
	__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers__v0 = 1U;
	__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers__v0 
	    = vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd;
    }
    // ALWAYS at VX_d_e_reg.v:145
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v0 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z
	   [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_a_reg_data
	   [3U]);
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v1 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z
	   [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_a_reg_data
	   [2U]);
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z
	   [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_a_reg_data
	   [1U]);
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v3 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z
	   [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_a_reg_data
	   [0U]);
    // ALWAYS at VX_csr_handler.v:45
    if (vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__is_csr) {
	vlTOPp->Vortex__DOT__vx_csr_handler__DOT____Vlvbound1 
	    = (0xfffU & vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__csr_result);
	if (VL_LIKELY((0x400U >= (0x7ffU & (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__csr_address))))) {
	    __Vdlyvval__Vortex__DOT__vx_csr_handler__DOT__csr__v0 
		= vlTOPp->Vortex__DOT__vx_csr_handler__DOT____Vlvbound1;
	    __Vdlyvset__Vortex__DOT__vx_csr_handler__DOT__csr__v0 = 1U;
	    __Vdlyvdim0__Vortex__DOT__vx_csr_handler__DOT__csr__v0 
		= (0x7ffU & (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__csr_address));
	}
    }
    // ALWAYS at VX_d_e_reg.v:145
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v0 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z
	   [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_b_reg_data
	   [3U]);
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v1 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z
	   [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_b_reg_data
	   [2U]);
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z
	   [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_b_reg_data
	   [1U]);
    __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v3 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__reg_data_z
	   [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_b_reg_data
	   [0U]);
    // ALWAYS at VX_register_file_slave.v:53
    if ((((((0U != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb)) 
	    & (0U != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd))) 
	   & vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid
	   [3U]) & (0U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__warp_num))) 
	 & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone)))) {
	__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v0 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data
	    [3U];
	__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v0 = 1U;
	__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v0 
	    = vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd;
    } else {
	if ((((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone) 
	      & ((3U == vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
		  [0U]) & (1U == (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall)))) 
	     & (0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v1 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x1fU];
	    __Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v1 = 1U;
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v2 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x1eU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v3 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x1dU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v4 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x1cU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v5 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x1bU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v6 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x1aU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v7 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x19U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v8 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x18U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v9 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x17U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v10 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x16U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v11 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x15U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v12 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x14U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v13 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x13U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v14 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x12U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v15 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x11U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v16 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0x10U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v17 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0xfU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v18 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0xeU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v19 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0xdU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v20 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0xcU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v21 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0xbU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v22 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0xaU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v23 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[9U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v24 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[8U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v25 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[7U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v26 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[6U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v27 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[5U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v28 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[4U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v29 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[3U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v30 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[2U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v31 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[1U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v32 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs
		[0U];
	}
    }
    // ALWAYS at VX_register_file_slave.v:53
    if ((((((0U != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb)) 
	    & (0U != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd))) 
	   & vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid
	   [2U]) & (0U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__warp_num))) 
	 & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone)))) {
	__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v0 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data
	    [2U];
	__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v0 = 1U;
	__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v0 
	    = vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd;
    } else {
	if ((((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone) 
	      & ((2U == vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
		  [0U]) & (1U == (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall)))) 
	     & (0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v1 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x1fU];
	    __Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v1 = 1U;
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v2 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x1eU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v3 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x1dU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v4 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x1cU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v5 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x1bU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v6 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x1aU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v7 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x19U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v8 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x18U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v9 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x17U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v10 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x16U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v11 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x15U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v12 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x14U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v13 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x13U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v14 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x12U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v15 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x11U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v16 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0x10U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v17 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0xfU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v18 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0xeU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v19 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0xdU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v20 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0xcU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v21 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0xbU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v22 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0xaU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v23 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[9U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v24 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[8U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v25 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[7U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v26 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[6U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v27 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[5U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v28 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[4U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v29 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[3U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v30 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[2U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v31 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[1U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v32 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs
		[0U];
	}
    }
    // ALWAYS at VX_register_file_slave.v:53
    if ((((((0U != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb)) 
	    & (0U != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd))) 
	   & vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid
	   [1U]) & (0U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__warp_num))) 
	 & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone)))) {
	__Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v0 
	    = vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data
	    [1U];
	__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v0 = 1U;
	__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v0 
	    = vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd;
    } else {
	if ((((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone) 
	      & ((1U == vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
		  [0U]) & (1U == (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall)))) 
	     & (0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)))) {
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v1 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x1fU];
	    __Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v1 = 1U;
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v2 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x1eU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v3 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x1dU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v4 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x1cU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v5 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x1bU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v6 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x1aU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v7 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x19U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v8 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x18U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v9 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x17U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v10 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x16U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v11 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x15U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v12 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x14U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v13 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x13U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v14 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x12U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v15 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x11U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v16 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0x10U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v17 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0xfU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v18 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0xeU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v19 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0xdU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v20 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0xcU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v21 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0xbU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v22 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0xaU];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v23 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[9U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v24 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[8U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v25 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[7U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v26 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[6U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v27 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[5U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v28 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[4U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v29 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[3U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v30 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[2U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v31 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[1U];
	    __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v32 
		= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs
		[0U];
	}
    }
    // ALWAYSPOST at VX_e_m_reg.v:139
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data[3U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v0;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data[2U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v1;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data[1U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v2;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data[0U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__b_reg_data__v3;
    // ALWAYSPOST at VX_m_w_reg.v:72
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid[3U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v0;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid[2U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v1;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid[1U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v2;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid[0U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__valid__v3;
    // ALWAYSPOST at VX_e_m_reg.v:148
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid[3U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v0;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid[2U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v1;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid[1U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v2;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid[0U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__valid__v3;
    // ALWAYSPOST at VX_m_w_reg.v:66
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result[3U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v0;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result[2U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v1;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result[1U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v2;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result[0U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__mem_result__v3;
    // ALWAYSPOST at VX_m_w_reg.v:65
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result[3U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v0;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result[2U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v1;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result[1U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v2;
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result[0U] 
	= __Vdlyvval__Vortex__DOT__vx_m_w_reg__DOT__alu_result__v3;
    // ALWAYSPOST at VX_e_m_reg.v:130
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result[3U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v0;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result[2U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v1;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result[1U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v2;
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result[0U] 
	= __Vdlyvval__Vortex__DOT__vx_e_m_reg__DOT__alu_result__v3;
    // ALWAYSPOST at VX_d_e_reg.v:167
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid[3U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v0;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid[2U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v1;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid[1U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v2;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid[0U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__valid__v3;
    // ALWAYSPOST at VX_register_file.v:48
    if (__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers__v0) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers[__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers__v0] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers__v0;
    }
    // ALWAYSPOST at VX_d_e_reg.v:150
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data[3U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v0;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data[2U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v1;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data[1U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v2;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data[0U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__a_reg_data__v3;
    // ALWAYSPOST at VX_csr_handler.v:48
    if (__Vdlyvset__Vortex__DOT__vx_csr_handler__DOT__csr__v0) {
	vlTOPp->Vortex__DOT__vx_csr_handler__DOT__csr[__Vdlyvdim0__Vortex__DOT__vx_csr_handler__DOT__csr__v0] 
	    = __Vdlyvval__Vortex__DOT__vx_csr_handler__DOT__csr__v0;
    }
    // ALWAYSPOST at VX_d_e_reg.v:151
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data[3U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v0;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data[2U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v1;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data[1U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v2;
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data[0U] 
	= __Vdlyvval__Vortex__DOT__vx_d_e_reg__DOT__b_reg_data__v3;
    // ALWAYSPOST at VX_register_file_slave.v:56
    if (__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v0) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v0] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v0;
    }
    if (__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v1) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x1fU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v1;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x1eU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v2;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x1dU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v3;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x1cU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v4;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x1bU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v5;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x1aU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v6;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x19U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v7;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x18U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v8;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x17U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v9;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x16U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v10;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x15U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v11;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x14U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v12;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x13U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v13;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x12U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v14;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x11U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v15;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0x10U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v16;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0xfU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v17;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0xeU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v18;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0xdU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v19;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0xcU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v20;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0xbU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v21;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0xaU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v22;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[9U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v23;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[8U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v24;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[7U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v25;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[6U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v26;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[5U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v27;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[4U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v28;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[3U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v29;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[2U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v30;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[1U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v31;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[0U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers__v32;
    }
    // ALWAYSPOST at VX_register_file_slave.v:56
    if (__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v0) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v0] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v0;
    }
    if (__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v1) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x1fU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v1;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x1eU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v2;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x1dU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v3;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x1cU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v4;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x1bU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v5;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x1aU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v6;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x19U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v7;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x18U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v8;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x17U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v9;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x16U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v10;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x15U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v11;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x14U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v12;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x13U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v13;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x12U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v14;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x11U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v15;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0x10U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v16;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0xfU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v17;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0xeU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v18;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0xdU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v19;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0xcU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v20;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0xbU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v21;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0xaU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v22;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[9U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v23;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[8U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v24;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[7U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v25;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[6U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v26;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[5U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v27;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[4U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v28;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[3U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v29;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[2U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v30;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[1U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v31;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[0U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers__v32;
    }
    // ALWAYSPOST at VX_register_file_slave.v:56
    if (__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v0) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[__Vdlyvdim0__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v0] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v0;
    }
    if (__Vdlyvset__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v1) {
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x1fU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v1;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x1eU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v2;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x1dU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v3;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x1cU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v4;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x1bU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v5;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x1aU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v6;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x19U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v7;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x18U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v8;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x17U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v9;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x16U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v10;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x15U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v11;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x14U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v12;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x13U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v13;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x12U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v14;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x11U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v15;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0x10U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v16;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0xfU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v17;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0xeU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v18;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0xdU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v19;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0xcU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v20;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0xbU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v21;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0xaU] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v22;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[9U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v23;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[8U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v24;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[7U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v25;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[6U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v26;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[5U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v27;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[4U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v28;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[3U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v29;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[2U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v30;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[1U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v31;
	vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[0U] 
	    = __Vdlyvval__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers__v32;
    }
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall 
	= __Vdly__Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall;
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__b_reg_data
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result[3U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result[2U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result[1U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result[0U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__mem_result
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result[3U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result[2U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result[1U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result[0U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__alu_result
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result[3U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result[2U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result[1U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result[0U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__alu_result
	[0U];
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__branch_type 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? 0U : (IData)(vlTOPp->Vortex__DOT__decode_branch_type));
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__jal = ((~ (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)) 
						 & (IData)(vlTOPp->Vortex__DOT__decode_jal));
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__valid
	[0U];
    vlTOPp->out_cache_driver_in_mem_write = vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__mem_write;
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__mem_write 
	= (7U & ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
		  ? 7U : ((0x23U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
			   ? (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			      >> 0xcU) : 7U)));
    vlTOPp->out_cache_driver_in_mem_read = vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__mem_read;
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__mem_read 
	= (7U & ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
		  ? 7U : ((3U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
			   ? (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			      >> 0xcU) : 7U)));
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC = 
	((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	  ? 0U : vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC);
    vlTOPp->Vortex__DOT__memory_branch_dest = (vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__curr_PC 
					       + (vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_offset 
						  << 1U));
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? 0xdeadbeefU : vlTOPp->Vortex__DOT__decode_itype_immed);
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__jal_offset 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? 0U : vlTOPp->Vortex__DOT__decode_jal_offset);
    vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data[0U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data[1U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data[2U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data[3U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next[0U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next[1U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next[2U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next[3U] 
	= vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__PC_next;
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next = vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data[3U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data[2U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data[1U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data[0U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__a_reg_data
	[0U];
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__is_csr = vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__is_csr;
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__csr_address 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__csr_address;
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__csr_result 
	= ((0xdU == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__csr_mask
	    : ((0xeU == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? (vlTOPp->Vortex__DOT__csr_decode_csr_data 
		   | vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__csr_mask)
	        : ((0xfU == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? (vlTOPp->Vortex__DOT__csr_decode_csr_data 
		       & ((IData)(0xffffffffU) - vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__csr_mask))
		    : 0xdeadbeefU)));
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__e_m_b_reg_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__e_m_b_reg_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__e_m_b_reg_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__e_m_b_reg_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__m_w_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid
	[3U];
    vlTOPp->Vortex__DOT__m_w_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid
	[2U];
    vlTOPp->Vortex__DOT__m_w_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid
	[1U];
    vlTOPp->Vortex__DOT__m_w_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_valid
	[0U];
    vlTOPp->Vortex__DOT__e_m_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid
	[3U];
    vlTOPp->Vortex__DOT__e_m_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid
	[2U];
    vlTOPp->Vortex__DOT__e_m_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid
	[1U];
    vlTOPp->Vortex__DOT__e_m_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_valid
	[0U];
    vlTOPp->Vortex__DOT__m_w_mem_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result
	[3U];
    vlTOPp->Vortex__DOT__m_w_mem_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result
	[2U];
    vlTOPp->Vortex__DOT__m_w_mem_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result
	[1U];
    vlTOPp->Vortex__DOT__m_w_mem_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result
	[0U];
    vlTOPp->Vortex__DOT__m_w_alu_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result
	[3U];
    vlTOPp->Vortex__DOT__m_w_alu_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result
	[2U];
    vlTOPp->Vortex__DOT__m_w_alu_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result
	[1U];
    vlTOPp->Vortex__DOT__m_w_alu_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result
	[0U];
    vlTOPp->Vortex__DOT__e_m_alu_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result
	[3U];
    vlTOPp->Vortex__DOT__e_m_alu_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result
	[2U];
    vlTOPp->Vortex__DOT__e_m_alu_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result
	[1U];
    vlTOPp->Vortex__DOT__e_m_alu_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result
	[0U];
    vlTOPp->Vortex__DOT__d_e_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid
	[3U];
    vlTOPp->Vortex__DOT__d_e_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid
	[2U];
    vlTOPp->Vortex__DOT__d_e_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid
	[1U];
    vlTOPp->Vortex__DOT__d_e_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs
	[0U];
    vlTOPp->Vortex__DOT__d_e_a_reg_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__d_e_a_reg_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__d_e_a_reg_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__d_e_a_reg_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__csr_decode_csr_data = ((0xc00U 
						 == (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address))
						 ? (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__cycle)
						 : 
						((0xc80U 
						  == (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address))
						  ? (IData)(
							    (vlTOPp->Vortex__DOT__vx_csr_handler__DOT__cycle 
							     >> 0x20U))
						  : 
						 ((0xc02U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address))
						   ? (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__instret)
						   : 
						  ((0xc82U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address))
						    ? (IData)(
							      (vlTOPp->Vortex__DOT__vx_csr_handler__DOT__instret 
							       >> 0x20U))
						    : 
						   ((0x400U 
						     >= 
						     (0x7ffU 
						      & (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address)))
						     ? 
						    vlTOPp->Vortex__DOT__vx_csr_handler__DOT__csr
						    [
						    (0x7ffU 
						     & (IData)(vlTOPp->Vortex__DOT__vx_csr_handler__DOT__decode_csr_address))]
						     : 0U)))));
    vlTOPp->Vortex__DOT__d_e_b_reg_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__d_e_b_reg_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__d_e_b_reg_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__d_e_b_reg_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2[3U] 
	= vlTOPp->Vortex__DOT__e_m_b_reg_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2[2U] 
	= vlTOPp->Vortex__DOT__e_m_b_reg_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2[1U] 
	= vlTOPp->Vortex__DOT__e_m_b_reg_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2[0U] 
	= vlTOPp->Vortex__DOT__e_m_b_reg_data[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid[3U] 
	= vlTOPp->Vortex__DOT__m_w_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid[2U] 
	= vlTOPp->Vortex__DOT__m_w_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid[1U] 
	= vlTOPp->Vortex__DOT__m_w_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid[0U] 
	= vlTOPp->Vortex__DOT__m_w_valid[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid[3U] 
	= vlTOPp->Vortex__DOT__e_m_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid[2U] 
	= vlTOPp->Vortex__DOT__e_m_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid[1U] 
	= vlTOPp->Vortex__DOT__e_m_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid[0U] 
	= vlTOPp->Vortex__DOT__e_m_valid[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result[3U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result[2U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result[1U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result[0U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data[3U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data[2U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data[1U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data[0U] 
	= vlTOPp->Vortex__DOT__m_w_mem_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result[3U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result[2U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result[1U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result[0U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result[3U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result[2U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result[1U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result[0U] 
	= vlTOPp->Vortex__DOT__m_w_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result[3U] 
	= vlTOPp->Vortex__DOT__e_m_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result[2U] 
	= vlTOPp->Vortex__DOT__e_m_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result[1U] 
	= vlTOPp->Vortex__DOT__e_m_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result[0U] 
	= vlTOPp->Vortex__DOT__e_m_alu_result[0U];
    vlTOPp->Vortex__DOT__execute_branch_stall = ((0U 
						  != (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__branch_type)) 
						 | (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__jal));
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid[3U] 
	= vlTOPp->Vortex__DOT__d_e_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid[2U] 
	= vlTOPp->Vortex__DOT__d_e_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid[1U] 
	= vlTOPp->Vortex__DOT__d_e_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid[0U] 
	= vlTOPp->Vortex__DOT__d_e_valid[0U];
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next[0U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next[1U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next[2U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next[3U] 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__PC_next;
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? 0U : ((IData)(4U) + vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC));
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data[3U] 
	= vlTOPp->Vortex__DOT__d_e_a_reg_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data[2U] 
	= vlTOPp->Vortex__DOT__d_e_a_reg_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data[1U] 
	= vlTOPp->Vortex__DOT__d_e_a_reg_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data[0U] 
	= vlTOPp->Vortex__DOT__d_e_a_reg_data[0U];
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__is_csr = 
	((~ (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)) 
	 & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_csr));
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__csr_address 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? 0U : (IData)(vlTOPp->Vortex__DOT__decode_csr_address));
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__csr_mask 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	    ? 0U : (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_csr) 
		     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			>> 0xeU)) ? (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					      >> 0xfU))
		     : vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
		    [0U]));
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op = 
	((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
	  ? 0xfU : (((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
		      >> 0x19U) & (0x33U == (0x7fU 
					     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)))
		     ? (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__mul_alu)
		     : (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_final_alu)));
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT__d_e_b_reg_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT__d_e_b_reg_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT__d_e_b_reg_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT__d_e_b_reg_data[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_rd2
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_wb_valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[0U];
    // ALWAYS at VX_memory.v:113
    vlTOPp->Vortex__DOT__memory_branch_dir = (1U & 
					      ((4U 
						& (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type))
					        ? (
						   (2U 
						    & (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type))
						    ? 
						   ((~ (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type)) 
						    & (~ 
						       (vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
							[0U] 
							>> 0x1fU)))
						    : 
						   ((1U 
						     & (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type))
						     ? 
						    (vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
						     [0U] 
						     >> 0x1fU)
						     : 
						    (~ 
						     (vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
						      [0U] 
						      >> 0x1fU))))
					        : (
						   (2U 
						    & (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type))
						    ? 
						   ((1U 
						     & (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type))
						     ? 
						    (vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
						     [0U] 
						     >> 0x1fU)
						     : 
						    (0U 
						     != 
						     vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
						     [0U]))
						    : 
						   ((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__branch_type) 
						    & (0U 
						       == 
						       vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
						       [0U])))));
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_alu_result
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rs2_src)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed
	    : vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	   [0U]);
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rs2_src)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed
	    : vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	   [1U]);
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rs2_src)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed
	    : vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	   [2U]);
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2 
	= ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rs2_src)
	    ? vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__itype_immed
	    : vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data
	   [3U]);
    vlTOPp->out_cache_driver_in_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data
	[3U];
    vlTOPp->out_cache_driver_in_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data
	[2U];
    vlTOPp->out_cache_driver_in_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data
	[1U];
    vlTOPp->out_cache_driver_in_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data
	[0U];
    vlTOPp->out_cache_driver_in_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid
	[3U];
    vlTOPp->out_cache_driver_in_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid
	[2U];
    vlTOPp->out_cache_driver_in_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid
	[1U];
    vlTOPp->out_cache_driver_in_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid
	[0U];
    vlTOPp->Vortex__DOT__memory_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid
	[3U];
    vlTOPp->Vortex__DOT__memory_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid
	[2U];
    vlTOPp->Vortex__DOT__memory_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid
	[1U];
    vlTOPp->Vortex__DOT__memory_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_valid
	[0U];
    vlTOPp->out_cache_driver_in_address[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address
	[3U];
    vlTOPp->out_cache_driver_in_address[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address
	[2U];
    vlTOPp->out_cache_driver_in_address[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address
	[1U];
    vlTOPp->out_cache_driver_in_address[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address
	[0U];
    vlTOPp->Vortex__DOT__memory_alu_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result
	[3U];
    vlTOPp->Vortex__DOT__memory_alu_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result
	[2U];
    vlTOPp->Vortex__DOT__memory_alu_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result
	[1U];
    vlTOPp->Vortex__DOT__memory_alu_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_alu_result
	[0U];
    vlTOPp->Vortex__DOT__execute_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid
	[3U];
    vlTOPp->Vortex__DOT__execute_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid
	[2U];
    vlTOPp->Vortex__DOT__execute_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid
	[1U];
    vlTOPp->Vortex__DOT__execute_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_valid
	[0U];
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next[0U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next[1U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next[2U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out;
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next[3U] 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__PC_next_out;
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1fU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1eU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1dU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1cU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1bU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x1aU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x19U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x18U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x17U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x16U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x15U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x14U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x13U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x12U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x11U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0x10U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xfU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xeU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xdU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xcU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xbU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0xaU];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[9U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[8U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[7U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[6U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[5U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[4U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__w0_t0_registers
	[0U];
    vlTOPp->Vortex__DOT__execute_b_reg_data[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__execute_b_reg_data[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__execute_b_reg_data[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__execute_b_reg_data[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_b_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__mult_signed_result 
	= VL_MULS_QQQ(64,64,64, VL_EXTENDS_QI(64,32, 
					      vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					      [0U]), 
		      VL_EXTENDS_QI(64,32, vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2));
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__mult_signed_result 
	= VL_MULS_QQQ(64,64,64, VL_EXTENDS_QI(64,32, 
					      vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					      [1U]), 
		      VL_EXTENDS_QI(64,32, vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2));
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__mult_signed_result 
	= VL_MULS_QQQ(64,64,64, VL_EXTENDS_QI(64,32, 
					      vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					      [2U]), 
		      VL_EXTENDS_QI(64,32, vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2));
    vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__mult_signed_result 
	= VL_MULS_QQQ(64,64,64, VL_EXTENDS_QI(64,32, 
					      vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					      [3U]), 
		      VL_EXTENDS_QI(64,32, vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2));
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid[3U] 
	= vlTOPp->Vortex__DOT__memory_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid[2U] 
	= vlTOPp->Vortex__DOT__memory_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid[1U] 
	= vlTOPp->Vortex__DOT__memory_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid[0U] 
	= vlTOPp->Vortex__DOT__memory_valid[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result[3U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result[2U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result[1U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result[0U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result[3U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result[2U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result[1U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result[0U] 
	= vlTOPp->Vortex__DOT__memory_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid[3U] 
	= vlTOPp->Vortex__DOT__execute_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid[2U] 
	= vlTOPp->Vortex__DOT__execute_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid[1U] 
	= vlTOPp->Vortex__DOT__execute_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid[0U] 
	= vlTOPp->Vortex__DOT__execute_valid[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1fU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1fU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1eU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1eU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1dU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1dU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1cU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1cU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1bU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1bU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x1aU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x1aU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x19U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x19U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x18U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x18U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x17U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x17U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x16U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x16U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x15U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x15U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x14U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x14U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x13U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x13U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x12U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x12U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x11U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x11U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0x10U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0x10U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xfU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xfU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xeU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xeU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xdU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xdU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xcU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xcU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xbU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xbU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0xaU] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0xaU];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[9U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[9U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[8U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[8U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[7U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[7U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[6U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[6U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[5U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[5U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[4U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[4U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_wspawn_regs[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data[3U] 
	= vlTOPp->Vortex__DOT__execute_b_reg_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data[2U] 
	= vlTOPp->Vortex__DOT__execute_b_reg_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data[1U] 
	= vlTOPp->Vortex__DOT__execute_b_reg_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data[0U] 
	= vlTOPp->Vortex__DOT__execute_b_reg_data[0U];
    // ALWAYS at VX_alu.v:48
    vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__0__KET____DOT__vx_alu__out_alu_result 
	= ((0x10U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	    ? ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? 0U : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			 ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [0U] : VL_MODDIV_III(32, 
							 vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							 [0U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [0U] : VL_MODDIVS_III(32, 
							  vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							  [0U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIV_III(32, 
					       vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					       [0U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIVS_III(32, 
						vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						[0U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))))
			 : ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((((QData)((IData)(
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [0U])) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U))
				 : (IData)((((((QData)((IData)(
							       VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
										[0U] 
										>> 0x1fU)))))) 
					       << 0x20U) 
					      | (QData)((IData)(
								vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
								[0U]))) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__mult_signed_result 
					    >> 0x20U))
				 : (IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__mult_signed_result)))))
	    : ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? vlTOPp->Vortex__DOT__csr_decode_csr_data
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? vlTOPp->Vortex__DOT__csr_decode_csr_data
			    : (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC 
			       + (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
				  << 0xcU)))) : ((2U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
						   << 0xcU)
						   : 
						  ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						    [0U] 
						    >= vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
						    ? 0U
						    : 0xffffffffU))
						  : 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2 
						   & vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [0U])
						   : 
						  (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [0U] 
						   | vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))))
	        : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? VL_SHIFTRS_III(32,32,5, 
					     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					     [0U], 
					     (0x1fU 
					      & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))
			    : (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [0U] >> (0x1fU & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)))
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [0U] ^ vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
			    : ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				[0U] < vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
			        ? 1U : 0U))) : ((2U 
						 & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						 ? 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (VL_LTS_III(1,32,32, 
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [0U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
						   ? 1U
						   : 0U)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [0U] 
						  << 
						  (0x1fU 
						   & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)))
						 : 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [0U] 
						  - vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [0U] 
						  + vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2))))));
    // ALWAYS at VX_alu.v:48
    vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__1__KET____DOT__vx_alu__out_alu_result 
	= ((0x10U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	    ? ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? 0U : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			 ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [1U] : VL_MODDIV_III(32, 
							 vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							 [1U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [1U] : VL_MODDIVS_III(32, 
							  vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							  [1U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIV_III(32, 
					       vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					       [1U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIVS_III(32, 
						vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						[1U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))))
			 : ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((((QData)((IData)(
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [1U])) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U))
				 : (IData)((((((QData)((IData)(
							       VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
										[1U] 
										>> 0x1fU)))))) 
					       << 0x20U) 
					      | (QData)((IData)(
								vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
								[1U]))) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__mult_signed_result 
					    >> 0x20U))
				 : (IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__mult_signed_result)))))
	    : ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? vlTOPp->Vortex__DOT__csr_decode_csr_data
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? vlTOPp->Vortex__DOT__csr_decode_csr_data
			    : (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC 
			       + (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
				  << 0xcU)))) : ((2U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
						   << 0xcU)
						   : 
						  ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						    [1U] 
						    >= vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
						    ? 0U
						    : 0xffffffffU))
						  : 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2 
						   & vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [1U])
						   : 
						  (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [1U] 
						   | vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))))
	        : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? VL_SHIFTRS_III(32,32,5, 
					     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					     [1U], 
					     (0x1fU 
					      & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))
			    : (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [1U] >> (0x1fU & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)))
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [1U] ^ vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
			    : ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				[1U] < vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
			        ? 1U : 0U))) : ((2U 
						 & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						 ? 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (VL_LTS_III(1,32,32, 
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [1U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
						   ? 1U
						   : 0U)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [1U] 
						  << 
						  (0x1fU 
						   & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)))
						 : 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [1U] 
						  - vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [1U] 
						  + vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2))))));
    // ALWAYS at VX_alu.v:48
    vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__2__KET____DOT__vx_alu__out_alu_result 
	= ((0x10U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	    ? ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? 0U : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			 ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [2U] : VL_MODDIV_III(32, 
							 vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							 [2U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [2U] : VL_MODDIVS_III(32, 
							  vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							  [2U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIV_III(32, 
					       vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					       [2U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIVS_III(32, 
						vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						[2U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))))
			 : ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((((QData)((IData)(
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [2U])) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U))
				 : (IData)((((((QData)((IData)(
							       VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
										[2U] 
										>> 0x1fU)))))) 
					       << 0x20U) 
					      | (QData)((IData)(
								vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
								[2U]))) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__mult_signed_result 
					    >> 0x20U))
				 : (IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__mult_signed_result)))))
	    : ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? vlTOPp->Vortex__DOT__csr_decode_csr_data
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? vlTOPp->Vortex__DOT__csr_decode_csr_data
			    : (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC 
			       + (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
				  << 0xcU)))) : ((2U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
						   << 0xcU)
						   : 
						  ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						    [2U] 
						    >= vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
						    ? 0U
						    : 0xffffffffU))
						  : 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2 
						   & vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [2U])
						   : 
						  (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [2U] 
						   | vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))))
	        : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? VL_SHIFTRS_III(32,32,5, 
					     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					     [2U], 
					     (0x1fU 
					      & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))
			    : (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [2U] >> (0x1fU & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)))
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [2U] ^ vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
			    : ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				[2U] < vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
			        ? 1U : 0U))) : ((2U 
						 & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						 ? 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (VL_LTS_III(1,32,32, 
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [2U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
						   ? 1U
						   : 0U)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [2U] 
						  << 
						  (0x1fU 
						   & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)))
						 : 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [2U] 
						  - vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [2U] 
						  + vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2))))));
    // ALWAYS at VX_alu.v:48
    vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__3__KET____DOT__vx_alu__out_alu_result 
	= ((0x10U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	    ? ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? 0U : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			 ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [3U] : VL_MODDIV_III(32, 
							 vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							 [3U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				    [3U] : VL_MODDIVS_III(32, 
							  vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							  [3U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIV_III(32, 
					       vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					       [3U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))
				 : ((0U == vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
				     ? 0xffffffffU : 
				    VL_DIVS_III(32, 
						vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						[3U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))))
			 : ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			     ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((((QData)((IData)(
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [3U])) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U))
				 : (IData)((((((QData)((IData)(
							       VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
										[3U] 
										>> 0x1fU)))))) 
					       << 0x20U) 
					      | (QData)((IData)(
								vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
								[3U]))) 
					     * (QData)((IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))) 
					    >> 0x20U)))
			     : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
				 ? (IData)((vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__mult_signed_result 
					    >> 0x20U))
				 : (IData)(vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__mult_signed_result)))))
	    : ((8U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
	        ? ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? vlTOPp->Vortex__DOT__csr_decode_csr_data
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? vlTOPp->Vortex__DOT__csr_decode_csr_data
			    : (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__curr_PC 
			       + (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
				  << 0xcU)))) : ((2U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__upper_immed 
						   << 0xcU)
						   : 
						  ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						    [3U] 
						    >= vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
						    ? 0U
						    : 0xffffffffU))
						  : 
						 ((1U 
						   & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2 
						   & vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [3U])
						   : 
						  (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						   [3U] 
						   | vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))))
	        : ((4U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		    ? ((2U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
		        ? ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? VL_SHIFTRS_III(32,32,5, 
					     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
					     [3U], 
					     (0x1fU 
					      & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))
			    : (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [3U] >> (0x1fU & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)))
		        : ((1U & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
			    ? (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
			       [3U] ^ vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
			    : ((vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
				[3U] < vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
			        ? 1U : 0U))) : ((2U 
						 & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						 ? 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (VL_LTS_III(1,32,32, 
							     vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
							     [3U], vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
						   ? 1U
						   : 0U)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [3U] 
						  << 
						  (0x1fU 
						   & vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)))
						 : 
						((1U 
						  & (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__alu_op))
						  ? 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [3U] 
						  - vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2)
						  : 
						 (vlTOPp->Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data
						  [3U] 
						  + vlTOPp->Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2))))));
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result[0U] 
	= vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__0__KET____DOT__vx_alu__out_alu_result;
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result[1U] 
	= vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__1__KET____DOT__vx_alu__out_alu_result;
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result[2U] 
	= vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__2__KET____DOT__vx_alu__out_alu_result;
    vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result[3U] 
	= vlTOPp->Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__3__KET____DOT__vx_alu__out_alu_result;
    vlTOPp->Vortex__DOT__execute_alu_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result
	[3U];
    vlTOPp->Vortex__DOT__execute_alu_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result
	[2U];
    vlTOPp->Vortex__DOT__execute_alu_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result
	[1U];
    vlTOPp->Vortex__DOT__execute_alu_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_execute__out_alu_result
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result[3U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result[2U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result[1U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result[0U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result[3U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result[2U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result[1U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result[0U] 
	= vlTOPp->Vortex__DOT__execute_alu_result[0U];
}

VL_INLINE_OPT void VVortex::_sequent__TOP__5(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_sequent__TOP__5\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid
	[0U];
    // ALWAYS at VX_m_w_reg.v:63
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd = vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__rd;
    // ALWAYS at VX_m_w_reg.v:63
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__warp_num 
	= vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num;
    // ALWAYS at VX_m_w_reg.v:63
    vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb = vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb;
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__rd = vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rd;
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num 
	= vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__warp_num;
    vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data[3U] 
	= ((3U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
	    ? vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data
	   [3U] : ((1U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
		    ? vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result
		   [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result
		   [3U]));
    vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data[2U] 
	= ((3U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
	    ? vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data
	   [2U] : ((1U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
		    ? vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result
		   [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result
		   [2U]));
    vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data[1U] 
	= ((3U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
	    ? vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data
	   [1U] : ((1U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
		    ? vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result
		   [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result
		   [1U]));
    vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data[0U] 
	= ((3U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
	    ? vlTOPp->Vortex__DOT__vx_writeback__DOT__out_pc_data
	   [0U] : ((1U == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
		    ? vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_alu_result
		   [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_writeback__in_mem_result
		   [0U]));
    // ALWAYS at VX_e_m_reg.v:128
    vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb = vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb;
    vlTOPp->Vortex__DOT__writeback_write_data[3U] = 
	vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data
	[3U];
    vlTOPp->Vortex__DOT__writeback_write_data[2U] = 
	vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data
	[2U];
    vlTOPp->Vortex__DOT__writeback_write_data[1U] = 
	vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data
	[1U];
    vlTOPp->Vortex__DOT__writeback_write_data[0U] = 
	vlTOPp->Vortex__DOT____Vcellout__vx_writeback__out_write_data
	[0U];
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rd = (0x1fU 
						& ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
						    ? 0U
						    : 
						   (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						    >> 7U)));
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__warp_num 
	= (0xfU & ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
		    ? 0U : (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data[3U] 
	= vlTOPp->Vortex__DOT__writeback_write_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data[2U] 
	= vlTOPp->Vortex__DOT__writeback_write_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data[1U] 
	= vlTOPp->Vortex__DOT__writeback_write_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data[0U] 
	= vlTOPp->Vortex__DOT__writeback_write_data
	[0U];
    // ALWAYS at VX_d_e_reg.v:145
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb = ((IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling)
						 ? 0U
						 : 
						(((((0x6fU 
						     == 
						     (0x7fU 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
						    | (0x67U 
						       == 
						       (0x7fU 
							& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))) 
						   | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs)) 
						  | ((0x73U 
						      == 
						      (0x7fU 
						       & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
						     & (0U 
							== 
							(7U 
							 & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							    >> 0xcU)))))
						  ? 3U
						  : 
						 ((3U 
						   == 
						   (0x7fU 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
						   ? 2U
						   : 
						  ((((((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_itype) 
						       | (0x33U 
							  == 
							  (0x7fU 
							   & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))) 
						      | (0x37U 
							 == 
							 (0x7fU 
							  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))) 
						     | (0x17U 
							== 
							(0x7fU 
							 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))) 
						    | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_csr))
						    ? 1U
						    : 0U))));
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_write_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_write_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_write_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_write_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data
	[0U];
}

VL_INLINE_OPT void VVortex::_sequent__TOP__6(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_sequent__TOP__6\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Variables
    // Begin mtask footprint  all: 
    VL_SIG8(__Vdly__Vortex__DOT__vx_fetch__DOT__warp_num,3,0);
    VL_SIG8(__Vdly__Vortex__DOT__vx_fetch__DOT__warp_count,3,0);
    VL_SIG8(__Vdlyvset__Vortex__DOT__vx_f_d_reg__DOT__valid__v0,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v4,0,0);
    VL_SIG8(__Vdlyvset__Vortex__DOT__vx_f_d_reg__DOT__valid__v4,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v5,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v6,0,0);
    VL_SIG8(__Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v7,0,0);
    // Body
    __Vdly__Vortex__DOT__vx_fetch__DOT__warp_count 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_count;
    __Vdly__Vortex__DOT__vx_fetch__DOT__warp_num = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num;
    __Vdlyvset__Vortex__DOT__vx_f_d_reg__DOT__valid__v0 = 0U;
    __Vdlyvset__Vortex__DOT__vx_f_d_reg__DOT__valid__v4 = 0U;
    // ALWAYS at VX_warp.v:71
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__real_PC 
	= ((IData)(vlTOPp->reset) ? 0U : ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_stall)
					   ? vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__temp_PC
					   : ((IData)(4U) 
					      + vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__temp_PC)));
    // ALWAYS at VX_f_d_reg.v:50
    if (vlTOPp->reset) {
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC = 0U;
    } else {
	if ((1U & (~ ((IData)(vlTOPp->Vortex__DOT__forwarding_fwd_stall) 
		      | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall))))) {
	    vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var;
	}
    }
    // ALWAYS at VX_warp.v:71
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__real_PC 
	= ((IData)(vlTOPp->reset) ? 0U : (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
					   & (7U == 
					      ((IData)(1U) 
					       + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state))))
					   ? vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
					  [0U] : ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_stall)
						   ? vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__temp_PC
						   : 
						  ((IData)(4U) 
						   + vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__temp_PC))));
    // ALWAYS at VX_warp.v:71
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__real_PC 
	= ((IData)(vlTOPp->reset) ? 0U : (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
					   & (6U == 
					      ((IData)(1U) 
					       + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state))))
					   ? vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
					  [0U] : ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_stall)
						   ? vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__temp_PC
						   : 
						  ((IData)(4U) 
						   + vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__temp_PC))));
    // ALWAYS at VX_warp.v:71
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__real_PC 
	= ((IData)(vlTOPp->reset) ? 0U : (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
					   & (5U == 
					      ((IData)(1U) 
					       + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state))))
					   ? vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
					  [0U] : ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_stall)
						   ? vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__temp_PC
						   : 
						  ((IData)(4U) 
						   + vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__temp_PC))));
    // ALWAYS at VX_warp.v:71
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__real_PC 
	= ((IData)(vlTOPp->reset) ? 0U : (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
					   & (4U == 
					      ((IData)(1U) 
					       + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state))))
					   ? vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
					  [0U] : ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_stall)
						   ? vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__temp_PC
						   : 
						  ((IData)(4U) 
						   + vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__temp_PC))));
    // ALWAYS at VX_warp.v:71
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__real_PC 
	= ((IData)(vlTOPp->reset) ? 0U : (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
					   & (3U == 
					      ((IData)(1U) 
					       + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state))))
					   ? vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
					  [0U] : ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_stall)
						   ? vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__temp_PC
						   : 
						  ((IData)(4U) 
						   + vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__temp_PC))));
    // ALWAYS at VX_warp.v:71
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__real_PC 
	= ((IData)(vlTOPp->reset) ? 0U : (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
					   & (2U == 
					      ((IData)(1U) 
					       + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state))))
					   ? vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
					  [0U] : ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_stall)
						   ? vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__temp_PC
						   : 
						  ((IData)(4U) 
						   + vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__temp_PC))));
    // ALWAYS at VX_warp.v:71
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__real_PC 
	= ((IData)(vlTOPp->reset) ? 0U : (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
					   & (1U == 
					      ((IData)(1U) 
					       + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state))))
					   ? vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_a_reg_data
					  [0U] : ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_stall)
						   ? vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__temp_PC
						   : 
						  ((IData)(4U) 
						   + vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__temp_PC))));
    // ALWAYS at VX_fetch.v:65
    __Vdly__Vortex__DOT__vx_fetch__DOT__warp_num = 
	(0xfU & (((((IData)(vlTOPp->reset) | ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num) 
					      >= (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state))) 
		   | (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp)) 
		  | (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__add_warp))
		  ? 0U : (vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_valid
			  [(7U & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)))]
			   ? ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))
			   : ((IData)(2U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)))));
    if (vlTOPp->Vortex__DOT__vx_fetch__DOT__add_warp) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state 
	    = (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)));
	__Vdly__Vortex__DOT__vx_fetch__DOT__warp_count 
	    = (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_count)));
    } else {
	if (vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp) {
	    __Vdly__Vortex__DOT__vx_fetch__DOT__warp_count 
		= (0xfU & ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_count) 
			   - (IData)(1U)));
	    if ((2U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_count))) {
		vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state = 0U;
	    }
	}
    }
    // ALWAYS at VX_f_d_reg.v:50
    if (vlTOPp->reset) {
	__Vdlyvset__Vortex__DOT__vx_f_d_reg__DOT__valid__v0 = 1U;
    } else {
	if ((1U & (~ ((IData)(vlTOPp->Vortex__DOT__forwarding_fwd_stall) 
		      | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall))))) {
	    __Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v4 
		= vlTOPp->Vortex__DOT____Vcellinp__vx_f_d_reg__in_valid
		[3U];
	    __Vdlyvset__Vortex__DOT__vx_f_d_reg__DOT__valid__v4 = 1U;
	    __Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v5 
		= vlTOPp->Vortex__DOT____Vcellinp__vx_f_d_reg__in_valid
		[2U];
	    __Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v6 
		= vlTOPp->Vortex__DOT____Vcellinp__vx_f_d_reg__in_valid
		[1U];
	    __Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v7 
		= vlTOPp->Vortex__DOT____Vcellinp__vx_f_d_reg__in_valid
		[0U];
	}
    }
    // ALWAYS at VX_f_d_reg.v:50
    if (vlTOPp->reset) {
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num = 0U;
    } else {
	if ((1U & (~ ((IData)(vlTOPp->Vortex__DOT__forwarding_fwd_stall) 
		      | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall))))) {
	    vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num 
		= vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num;
	}
    }
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_count 
	= __Vdly__Vortex__DOT__vx_fetch__DOT__warp_count;
    // ALWAYSPOST at VX_f_d_reg.v:56
    if (__Vdlyvset__Vortex__DOT__vx_f_d_reg__DOT__valid__v0) {
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid[0U] = 0U;
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid[1U] = 0U;
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid[2U] = 0U;
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid[3U] = 0U;
    }
    if (__Vdlyvset__Vortex__DOT__vx_f_d_reg__DOT__valid__v4) {
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid[3U] 
	    = __Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v4;
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid[2U] 
	    = __Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v5;
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid[1U] 
	    = __Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v6;
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid[0U] 
	    = __Vdlyvval__Vortex__DOT__vx_f_d_reg__DOT__valid__v7;
    }
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num = __Vdly__Vortex__DOT__vx_fetch__DOT__warp_num;
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (0U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (0U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (7U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (7U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (6U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (6U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (5U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (5U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (4U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (4U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__real_PC));
    // ALWAYS at VX_warp.v:57
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__temp_PC 
	= (((IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal) 
	    & (1U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	    ? vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__jal_dest
	    : (((IData)(vlTOPp->Vortex__DOT__memory_branch_dir) 
		& (1U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)))
	        ? vlTOPp->Vortex__DOT__memory_branch_dest
	        : vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__real_PC));
    vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid[3U] 
	= vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid[2U] 
	= vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid[1U] 
	= vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid[0U] 
	= vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__valid
	[0U];
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[0U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[7U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[6U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[5U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[4U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[3U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[2U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc[1U] 
	= vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__temp_PC;
    vlTOPp->Vortex__DOT__f_d_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid
	[3U];
    vlTOPp->Vortex__DOT__f_d_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid
	[2U];
    vlTOPp->Vortex__DOT__f_d_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid
	[1U];
    vlTOPp->Vortex__DOT__f_d_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_f_d_reg__out_valid
	[0U];
    // ALWAYS at VX_fetch.v:172
    if ((0U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [0U];
    }
    if ((1U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [1U];
    }
    if ((2U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [2U];
    }
    if ((3U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [3U];
    }
    if ((4U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [4U];
    }
    if ((5U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [5U];
    }
    if ((6U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [6U];
    }
    if ((7U == (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num))) {
	vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var 
	    = vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_glob_pc
	    [7U];
    }
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid[3U] 
	= vlTOPp->Vortex__DOT__f_d_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid[2U] 
	= vlTOPp->Vortex__DOT__f_d_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid[1U] 
	= vlTOPp->Vortex__DOT__f_d_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid[0U] 
	= vlTOPp->Vortex__DOT__f_d_valid[0U];
    vlTOPp->icache_request_pc_address = vlTOPp->Vortex__DOT__vx_fetch__DOT__out_PC_var;
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
	[0U];
    vlTOPp->Vortex__DOT__decode_valid[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid
	[3U];
    vlTOPp->Vortex__DOT__decode_valid[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid
	[2U];
    vlTOPp->Vortex__DOT__decode_valid[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid
	[1U];
    vlTOPp->Vortex__DOT__decode_valid[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_decode__out_valid
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid[3U] 
	= vlTOPp->Vortex__DOT__decode_valid[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid[2U] 
	= vlTOPp->Vortex__DOT__decode_valid[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid[1U] 
	= vlTOPp->Vortex__DOT__decode_valid[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid[0U] 
	= vlTOPp->Vortex__DOT__decode_valid[0U];
}

VL_INLINE_OPT void VVortex::_sequent__TOP__7(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_sequent__TOP__7\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    // ALWAYS at VX_register_file_slave.v:68
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__out_src1_data 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers
	[(0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
		   >> 0xfU))];
    // ALWAYS at VX_register_file_slave.v:68
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__out_src1_data 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers
	[(0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
		   >> 0xfU))];
    // ALWAYS at VX_register_file_slave.v:68
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__out_src1_data 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers
	[(0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
		   >> 0xfU))];
    // ALWAYS at VX_register_file.v:52
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_src1_data 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[(0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
		   >> 0xfU))];
    // ALWAYS at VX_register_file_slave.v:68
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__out_src2_data 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers
	[(0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
		   >> 0x14U))];
    // ALWAYS at VX_register_file_slave.v:68
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__out_src2_data 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers
	[(0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
		   >> 0x14U))];
    // ALWAYS at VX_register_file_slave.v:68
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__out_src2_data 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers
	[(0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
		   >> 0x14U))];
    // ALWAYS at VX_register_file.v:52
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_src2_data 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers
	[(0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
		   >> 0x14U))];
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__out_src1_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__out_src1_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__out_src1_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_src1_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__out_src2_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__out_src2_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__out_src2_data;
    vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_src2_data;
}

VL_INLINE_OPT void VVortex::_combo__TOP__8(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_combo__TOP__8\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data[3U] 
	= vlTOPp->in_cache_driver_out_data[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data[2U] 
	= vlTOPp->in_cache_driver_out_data[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data[1U] 
	= vlTOPp->in_cache_driver_out_data[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data[0U] 
	= vlTOPp->in_cache_driver_out_data[0U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data
	[0U];
    vlTOPp->Vortex__DOT__memory_mem_result[3U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result
	[3U];
    vlTOPp->Vortex__DOT__memory_mem_result[2U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result
	[2U];
    vlTOPp->Vortex__DOT__memory_mem_result[1U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result
	[1U];
    vlTOPp->Vortex__DOT__memory_mem_result[0U] = vlTOPp->Vortex__DOT____Vcellout__vx_memory__out_mem_result
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result[3U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result[2U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result[1U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result[0U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data[3U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data[2U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data[1U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data[0U] 
	= vlTOPp->Vortex__DOT__memory_mem_result[0U];
}

VL_INLINE_OPT void VVortex::_sequent__TOP__9(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_sequent__TOP__9\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    // ALWAYS at VX_f_d_reg.v:50
    if (vlTOPp->reset) {
	vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction = 0U;
    } else {
	if ((1U & (~ ((IData)(vlTOPp->Vortex__DOT__forwarding_fwd_stall) 
		      | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall))))) {
	    vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
		= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall)
		    ? 0U : vlTOPp->icache_response_instruction);
	}
    }
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_itype = 
	((0x13U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	 | (3U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)));
    vlTOPp->Vortex__DOT__decode_csr_address = (0xfffU 
					       & (((0U 
						    != 
						    (7U 
						     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							>> 0xcU))) 
						   & (2U 
						      <= 
						      (0xfffU 
						       & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							  >> 0x14U))))
						   ? 
						  (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						   >> 0x14U)
						   : 0x55U));
    // ALWAYS at VX_decode.v:577
    vlTOPp->__Vtableidx1 = (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				  >> 0xcU));
    vlTOPp->Vortex__DOT__vx_decode__DOT__mul_alu = 
	vlTOPp->__Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu
	[vlTOPp->__Vtableidx1];
    vlTOPp->Vortex__DOT__vx_decode__DOT__alu_tempp 
	= (0xfffU & (((1U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				    >> 0xcU))) | (5U 
						  == 
						  (7U 
						   & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						      >> 0xcU))))
		      ? (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				  >> 0x14U)) : (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						>> 0x14U)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_csr = (
						   (0x73U 
						    == 
						    (0x7fU 
						     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
						   & (0U 
						      != 
						      (7U 
						       & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							  >> 0xcU))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__jal_sys_jal 
	= ((0U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0xcU))) & (2U > (0xfffU 
					     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						>> 0x14U))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt = 
	((0x6bU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	 & (4U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0xcU))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn 
	= ((0x6bU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	   & (0U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			   >> 0xcU))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone = 
	((0x6bU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	 & (5U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0xcU))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs = 
	((0x6bU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	 & (6U == (7U & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0xcU))));
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd 
	= (((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			>> 0xfU)) == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rd)) 
	     & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				>> 0xfU)))) & (0U != (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num) 
	      == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd 
	= (((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			>> 0x14U)) == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__rd)) 
	     & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				>> 0x14U)))) & (0U 
						!= (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num) 
	      == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__warp_num)));
    // ALWAYS at VX_decode.v:508
    vlTOPp->Vortex__DOT__decode_itype_immed = ((0x40U 
						& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
					        ? (
						   (0x20U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 
						   ((0x10U 
						     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						     ? 0xdeadbeefU
						     : 
						    ((8U 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						      ? 0xdeadbeefU
						      : 
						     ((4U 
						       & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						       ? 0xdeadbeefU
						       : 
						      ((2U 
							& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						        ? 
						       ((1U 
							 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
							 ? 
							((0xfffff000U 
							  & (VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
										>> 0x1fU)))) 
							     << 0xcU)) 
							 | ((0x800U 
							     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								>> 0x14U)) 
							    | ((0x400U 
								& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								   << 3U)) 
							       | ((0x3f0U 
								   & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								      >> 0x15U)) 
								  | (0xfU 
								     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
									>> 8U))))))
							 : 0xdeadbeefU)
						        : 0xdeadbeefU))))
						    : 0xdeadbeefU)
					        : (
						   (0x20U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 
						   ((0x10U 
						     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						     ? 0xdeadbeefU
						     : 
						    ((8U 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						      ? 0xdeadbeefU
						      : 
						     ((4U 
						       & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						       ? 0xdeadbeefU
						       : 
						      ((2U 
							& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						        ? 
						       ((1U 
							 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
							 ? 
							((0xfffff000U 
							  & (VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
										>> 0x1fU)))) 
							     << 0xcU)) 
							 | ((0xfe0U 
							     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								>> 0x14U)) 
							    | (0x1fU 
							       & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								  >> 7U))))
							 : 0xdeadbeefU)
						        : 0xdeadbeefU))))
						    : 
						   ((0x10U 
						     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						     ? 
						    ((8U 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						      ? 0xdeadbeefU
						      : 
						     ((4U 
						       & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						       ? 0xdeadbeefU
						       : 
						      ((2U 
							& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						        ? 
						       ((1U 
							 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
							 ? 
							((0xfffff000U 
							  & (VL_NEGATE_I((IData)(
										(1U 
										& ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__alu_tempp) 
										>> 0xbU)))) 
							     << 0xcU)) 
							 | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__alu_tempp))
							 : 0xdeadbeefU)
						        : 0xdeadbeefU)))
						     : 
						    ((8U 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						      ? 0xdeadbeefU
						      : 
						     ((4U 
						       & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						       ? 0xdeadbeefU
						       : 
						      ((2U 
							& vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						        ? 
						       ((1U 
							 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
							 ? 
							((0xfffff000U 
							  & (VL_NEGATE_I((IData)(
										(1U 
										& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
										>> 0x1fU)))) 
							     << 0xcU)) 
							 | (0xfffU 
							    & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							       >> 0x14U)))
							 : 0xdeadbeefU)
						        : 0xdeadbeefU))))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__is_ebreak 
	= ((0x73U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__jal_sys_jal) 
	      & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
	      [0U]));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (1U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (2U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (3U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (4U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (5U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (6U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__real_wspawn 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	   & (7U == (0xfU & ((IData)(1U) + (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_state)))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (1U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (2U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (3U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (4U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (5U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (6U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__real_isclone 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_clone) 
	   & (7U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    // ALWAYS at VX_decode.v:451
    if ((0x40U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	if ((0x20U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	    if ((0x10U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		vlTOPp->Vortex__DOT__decode_jal_offset 
		    = ((8U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
		        ? 0xdeadbeefU : ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
					  ? 0xdeadbeefU
					  : ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
					      ? ((1U 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						  ? 
						 (((0U 
						    == 
						    (7U 
						     & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							>> 0xcU))) 
						   & (2U 
						      > 
						      (0xfffU 
						       & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							  >> 0x14U))))
						   ? 0xb0000000U
						   : 0xdeadbeefU)
						  : 0xdeadbeefU)
					      : 0xdeadbeefU)));
	    } else {
		if ((8U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		    if ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			vlTOPp->Vortex__DOT__decode_jal_offset 
			    = ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
			        ? ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
				    ? ((0xffe00000U 
					& (VL_NEGATE_I((IData)(
							       (1U 
								& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								   >> 0x1fU)))) 
					   << 0x15U)) 
				       | ((0x100000U 
					   & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					      >> 0xbU)) 
					  | ((0xff000U 
					      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction) 
					     | ((0x800U 
						 & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						    >> 9U)) 
						| (0x7feU 
						   & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						      >> 0x14U))))))
				    : 0xdeadbeefU) : 0xdeadbeefU);
		    } else {
			if ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			    if ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
				if (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs) 
				     | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt))) {
				    vlTOPp->Vortex__DOT__decode_jal_offset = 0U;
				}
			    } else {
				vlTOPp->Vortex__DOT__decode_jal_offset = 0xdeadbeefU;
			    }
			} else {
			    vlTOPp->Vortex__DOT__decode_jal_offset = 0xdeadbeefU;
			}
		    }
		} else {
		    vlTOPp->Vortex__DOT__decode_jal_offset 
			= ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
			    ? ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
			        ? ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
				    ? ((0xfffff000U 
					& (VL_NEGATE_I((IData)(
							       (1U 
								& (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
								   >> 0x1fU)))) 
					   << 0xcU)) 
				       | (0xfffU & 
					  (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					   >> 0x14U)))
				    : 0xdeadbeefU) : 0xdeadbeefU)
			    : 0xdeadbeefU);
		}
	    }
	} else {
	    vlTOPp->Vortex__DOT__decode_jal_offset = 0xdeadbeefU;
	}
    } else {
	vlTOPp->Vortex__DOT__decode_jal_offset = 0xdeadbeefU;
    }
    // ALWAYS at VX_decode.v:451
    if ((0x40U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	if ((0x20U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	    if ((0x10U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		vlTOPp->Vortex__DOT__decode_jal = (
						   (~ 
						    (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						     >> 3U)) 
						   & ((~ 
						       (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							>> 2U)) 
						      & ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							  >> 1U) 
							 & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							    & ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__jal_sys_jal) 
							       & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
							       [0U])))));
	    } else {
		if ((8U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		    if ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			vlTOPp->Vortex__DOT__decode_jal 
			    = ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				>> 1U) & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					  & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
					  [0U]));
		    } else {
			if ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			    if ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
				if (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs) 
				     | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt))) {
				    vlTOPp->Vortex__DOT__decode_jal 
					= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
					[0U];
				}
			    } else {
				vlTOPp->Vortex__DOT__decode_jal = 0U;
			    }
			} else {
			    vlTOPp->Vortex__DOT__decode_jal = 0U;
			}
		    }
		} else {
		    vlTOPp->Vortex__DOT__decode_jal 
			= ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			    >> 2U) & ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				       >> 1U) & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						 & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
						 [0U])));
		}
	    }
	} else {
	    vlTOPp->Vortex__DOT__decode_jal = 0U;
	}
    } else {
	vlTOPp->Vortex__DOT__decode_jal = 0U;
    }
    // ALWAYS at VX_decode.v:519
    if ((0x40U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	if ((0x20U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	    if ((0x10U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		vlTOPp->Vortex__DOT__decode_branch_type = 0U;
	    } else {
		if ((8U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		    if ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			vlTOPp->Vortex__DOT__decode_branch_type = 0U;
		    } else {
			if ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			    if ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
				if (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs) 
				     | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt))) {
				    vlTOPp->Vortex__DOT__decode_branch_type = 0U;
				}
			    } else {
				vlTOPp->Vortex__DOT__decode_branch_type = 0U;
			    }
			} else {
			    vlTOPp->Vortex__DOT__decode_branch_type = 0U;
			}
		    }
		} else {
		    vlTOPp->Vortex__DOT__decode_branch_type 
			= ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
			    ? 0U : ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
				     ? ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
					 ? ((0x4000U 
					     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
					     ? ((0x2000U 
						 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						 ? 
						((0x1000U 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						  ? 6U
						  : 5U)
						 : 
						((0x1000U 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						  ? 4U
						  : 3U))
					     : ((0x2000U 
						 & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						 ? 0U
						 : 
						((0x1000U 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						  ? 2U
						  : 1U)))
					 : 0U) : 0U));
		}
	    }
	} else {
	    vlTOPp->Vortex__DOT__decode_branch_type = 0U;
	}
    } else {
	vlTOPp->Vortex__DOT__decode_branch_type = 0U;
    }
    // ALWAYS at VX_decode.v:519
    if ((0x40U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	if ((0x20U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
	    if ((0x10U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		vlTOPp->Vortex__DOT__decode_branch_stall = 0U;
	    } else {
		if ((8U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
		    if ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			vlTOPp->Vortex__DOT__decode_branch_stall 
			    = ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				>> 1U) & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					  & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
					  [0U]));
		    } else {
			if ((2U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
			    if ((1U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) {
				if (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs) 
				     | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt))) {
				    vlTOPp->Vortex__DOT__decode_branch_stall 
					= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
					[0U];
				}
			    } else {
				vlTOPp->Vortex__DOT__decode_branch_stall = 0U;
			    }
			} else {
			    vlTOPp->Vortex__DOT__decode_branch_stall = 0U;
			}
		    }
		} else {
		    vlTOPp->Vortex__DOT__decode_branch_stall 
			= ((4U & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
			    ? ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				>> 1U) & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
					  & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
					  [0U])) : 
			   ((vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			     >> 1U) & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				       & vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_valid
				       [0U])));
		}
	    }
	} else {
	    vlTOPp->Vortex__DOT__decode_branch_stall = 0U;
	}
    } else {
	vlTOPp->Vortex__DOT__decode_branch_stall = 0U;
    }
    vlTOPp->Vortex__DOT__decode_change_mask = ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jalrs) 
					       | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_jmprt));
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd 
	= ((((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0xfU)) == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__rd)) 
	      & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				 >> 0xfU)))) & (0U 
						!= (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))) 
	    & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd))) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num) 
	      == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd 
	= ((((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			 >> 0x14U)) == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__rd)) 
	      & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				 >> 0x14U)))) & (0U 
						 != (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))) 
	    & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd))) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num) 
	      == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__warp_num)));
    vlTOPp->out_ebreak = ((0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)) 
			  & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_ebreak));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[0U] 
	= (((0U == (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall)) 
	    | (1U != (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall))) 
	   & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__real_zero_isclone));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[1U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[2U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[3U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[4U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[5U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[6U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall[7U] 
	= ((((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.__PVT__clone_state_stall)) 
	     | (1U != (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.__PVT__clone_state_stall))) 
	    & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__real_isclone)) 
	   | (((0U == (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall)) 
	       & (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__real_wspawn)) 
	      | (1U < (IData)(vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.__PVT__wspawn_state_stall))));
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_final_alu 
	= ((0x63U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
	    ? ((5U > (IData)(vlTOPp->Vortex__DOT__decode_branch_type))
	        ? 1U : 0xaU) : ((0x37U == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
				 ? 0xbU : ((0x17U == 
					    (0x7fU 
					     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
					    ? 0xcU : 
					   ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_csr)
					     ? ((1U 
						 == 
						 (3U 
						  & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						     >> 0xcU)))
						 ? 0xdU
						 : 
						((2U 
						  == 
						  (3U 
						   & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
						      >> 0xcU)))
						  ? 0xeU
						  : 0xfU))
					     : (((0x23U 
						  == 
						  (0x7fU 
						   & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)) 
						 | (3U 
						    == 
						    (0x7fU 
						     & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)))
						 ? 0U
						 : 
						((0x4000U 
						  & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						  ? 
						 ((0x2000U 
						   & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						   ? 
						  ((0x1000U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 9U
						    : 8U)
						   : 
						  ((0x1000U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 
						   ((0U 
						     == 
						     (0x7fU 
						      & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							 >> 0x19U)))
						     ? 6U
						     : 7U)
						    : 5U))
						  : 
						 ((0x2000U 
						   & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						   ? 
						  ((0x1000U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 4U
						    : 3U)
						   : 
						  ((0x1000U 
						    & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction)
						    ? 2U
						    : 
						   ((0x13U 
						     == 
						     (0x7fU 
						      & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
						     ? 0U
						     : 
						    ((0U 
						      == 
						      (0x7fU 
						       & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
							  >> 0x19U)))
						      ? 0U
						      : 1U))))))))));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (0U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (1U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (2U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (3U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (4U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (5U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (6U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_change_mask 
	= ((IData)(vlTOPp->Vortex__DOT__decode_change_mask) 
	   & (7U == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd 
	= (((((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			  >> 0xfU)) == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd)) 
	       & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				  >> 0xfU)))) & (0U 
						 != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))) 
	     & ((IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__warp_num) 
		== (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num))) 
	    & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd))) 
	   & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)));
    vlTOPp->Vortex__DOT__forwarding_fwd_stall = ((((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd) 
						   | (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd)) 
						  & (2U 
						     == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))) 
						 | (((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd) 
						     | (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)) 
						    & (2U 
						       == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))));
    vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd 
	= (((((((0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
			  >> 0x14U)) == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__rd)) 
	       & (0U != (0x1fU & (vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction 
				  >> 0x14U)))) & (0U 
						  != (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))) 
	     & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd))) 
	    & (~ (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd))) 
	   & ((IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__warp_num) 
	      == (IData)(vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__warp_num)));
    // ALWAYS at VX_decode.v:276
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [1U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [2U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [3U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [4U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [5U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [6U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
	   | vlTOPp->Vortex__DOT__vx_decode__DOT__glob_clone_stall
	   [7U]);
    vlTOPp->Vortex__DOT__forwarding_src1_fwd = (((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd) 
						 | (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)) 
						| (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd));
    vlTOPp->Vortex__DOT__forwarding_src2_fwd = (((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd) 
						 | (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)) 
						| (IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__add_warp = 
	(((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn) 
	  & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_ebreak))) 
	 & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall)));
    vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__stalling 
	= (((IData)(vlTOPp->Vortex__DOT__forwarding_fwd_stall) 
	    | (IData)(vlTOPp->Vortex__DOT__execute_branch_stall)) 
	   | (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__stall = ((((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall) 
						   | (IData)(vlTOPp->Vortex__DOT__decode_branch_stall)) 
						  | (IData)(vlTOPp->Vortex__DOT__forwarding_fwd_stall)) 
						 | (IData)(vlTOPp->Vortex__DOT__execute_branch_stall));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__remove_warp 
	= (((IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_ebreak) 
	    & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__is_wspawn))) 
	   & (~ (IData)(vlTOPp->Vortex__DOT__vx_decode__DOT__temp_out_clone_stall)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (0U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (1U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (2U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (3U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (4U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (5U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (6U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
    vlTOPp->Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_stall 
	= ((IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__stall) 
	   | (7U != (IData)(vlTOPp->Vortex__DOT__vx_fetch__DOT__warp_num)));
}

VL_INLINE_OPT void VVortex::_combo__TOP__10(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_combo__TOP__10\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [3U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [3U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [3U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [3U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [3U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [3U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [3U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [2U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [2U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [2U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [2U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [2U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [2U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [2U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [1U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [1U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [1U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [1U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [1U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [1U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [1U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [0U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [0U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [0U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [0U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [0U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [0U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [0U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [3U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [3U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [3U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [3U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [3U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [3U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [3U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [3U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [2U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [2U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [2U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [2U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [2U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [2U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [2U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [2U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [1U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [1U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [1U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [1U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [1U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [1U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [1U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [1U])));
    vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd)
	    ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_d_e_reg__DOT__wb))
	        ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next
	       [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
	       [0U]) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd)
			 ? ((3U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
			     ? vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next
			    [0U] : ((2U == (IData)(vlTOPp->Vortex__DOT__vx_e_m_reg__DOT__wb))
				     ? vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data
				    [0U] : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result
				    [0U])) : ((IData)(vlTOPp->Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd)
					       ? ((3U 
						   == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						   ? 
						  vlTOPp->Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next
						  [0U]
						   : 
						  ((2U 
						    == (IData)(vlTOPp->Vortex__DOT__vx_m_w_reg__DOT__wb))
						    ? 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data
						   [0U]
						    : 
						   vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result
						   [0U]))
					       : vlTOPp->Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result
					      [0U])));
    vlTOPp->Vortex__DOT__forwarding_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__forwarding_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__forwarding_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__forwarding_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__forwarding_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__forwarding_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__forwarding_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__forwarding_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__forwarding_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__forwarding_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__forwarding_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__forwarding_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__forwarding_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__forwarding_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__forwarding_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__forwarding_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data[0U] 
	= ((0x6fU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
	    ? vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC
	    : ((IData)(vlTOPp->Vortex__DOT__forwarding_src1_fwd)
	        ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data
	       [0U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
	       [0U]));
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data[1U] 
	= ((0x6fU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
	    ? vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC
	    : ((IData)(vlTOPp->Vortex__DOT__forwarding_src1_fwd)
	        ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data
	       [1U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
	       [1U]));
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data[2U] 
	= ((0x6fU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
	    ? vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC
	    : ((IData)(vlTOPp->Vortex__DOT__forwarding_src1_fwd)
	        ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data
	       [2U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
	       [2U]));
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data[3U] 
	= ((0x6fU == (0x7fU & vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__instruction))
	    ? vlTOPp->Vortex__DOT__vx_f_d_reg__DOT__curr_PC
	    : ((IData)(vlTOPp->Vortex__DOT__forwarding_src1_fwd)
	        ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data
	       [3U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register
	       [3U]));
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src1_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src1_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src1_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src1_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data[0U] 
	= ((IData)(vlTOPp->Vortex__DOT__forwarding_src2_fwd)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data
	   [0U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register
	   [0U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data[1U] 
	= ((IData)(vlTOPp->Vortex__DOT__forwarding_src2_fwd)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data
	   [1U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register
	   [1U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data[2U] 
	= ((IData)(vlTOPp->Vortex__DOT__forwarding_src2_fwd)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data
	   [2U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register
	   [2U]);
    vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data[3U] 
	= ((IData)(vlTOPp->Vortex__DOT__forwarding_src2_fwd)
	    ? vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data
	   [3U] : vlTOPp->Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register
	   [3U]);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src2_fwd_data[3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data
	[3U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src2_fwd_data[2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data
	[2U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src2_fwd_data[1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data
	[1U];
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one.in_src2_fwd_data[0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[0U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[0U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[0U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_a_reg_data[0U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data
	[0U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[0U][3U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data
	[3U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[0U][2U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data
	[2U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[0U][1U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data
	[1U];
    vlTOPp->Vortex__DOT__vx_decode__DOT__glob_b_reg_data[0U][0U] 
	= vlTOPp->Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data
	[0U];
}

void VVortex::_eval(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_eval\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    if (((IData)(vlTOPp->clk) & (~ (IData)(vlTOPp->__Vclklast__TOP__clk)))) {
	vlTOPp->_sequent__TOP__4(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__15(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one__16(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one__17(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one__18(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one__19(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one__20(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one__21(vlSymsp);
	vlTOPp->_sequent__TOP__5(vlSymsp);
    }
    if ((((IData)(vlTOPp->clk) & (~ (IData)(vlTOPp->__Vclklast__TOP__clk))) 
	 | ((IData)(vlTOPp->reset) & (~ (IData)(vlTOPp->__Vclklast__TOP__reset))))) {
	vlTOPp->_sequent__TOP__6(vlSymsp);
    }
    if (((~ (IData)(vlTOPp->clk)) & (IData)(vlTOPp->__Vclklast__TOP__clk))) {
	vlTOPp->_sequent__TOP__7(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__22(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__22(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__22(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__22(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__22(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__22(vlSymsp);
	vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one._sequent__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__22(vlSymsp);
    }
    vlTOPp->_combo__TOP__8(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__29(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__29(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__29(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__29(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__29(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__29(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__29(vlSymsp);
    if ((((IData)(vlTOPp->clk) & (~ (IData)(vlTOPp->__Vclklast__TOP__clk))) 
	 | ((IData)(vlTOPp->reset) & (~ (IData)(vlTOPp->__Vclklast__TOP__reset))))) {
	vlTOPp->_sequent__TOP__9(vlSymsp);
    }
    vlTOPp->_combo__TOP__10(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__36(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__36(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__36(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__36(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__36(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__36(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one._combo__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__36(vlSymsp);
    vlTOPp->_settle__TOP__3(vlSymsp);
    // Final
    vlTOPp->__Vclklast__TOP__clk = vlTOPp->clk;
    vlTOPp->__Vclklast__TOP__reset = vlTOPp->reset;
}

void VVortex::_eval_initial(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_eval_initial\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    vlTOPp->_initial__TOP__1(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one._initial__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__1(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one._initial__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__1(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one._initial__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__1(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one._initial__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__1(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one._initial__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__1(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one._initial__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__1(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one._initial__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__1(vlSymsp);
    vlTOPp->__Vclklast__TOP__clk = vlTOPp->clk;
    vlTOPp->__Vclklast__TOP__reset = vlTOPp->reset;
}

void VVortex::final() {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::final\n"); );
    // Variables
    VVortex__Syms* __restrict vlSymsp = this->__VlSymsp;
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
}

void VVortex::_eval_settle(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_eval_settle\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    vlTOPp->_settle__TOP__2(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one._settle__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__8(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__VX_Context_one._settle__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__8(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__VX_Context_one._settle__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__8(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__VX_Context_one._settle__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__8(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__VX_Context_one._settle__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__8(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__VX_Context_one._settle__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__8(vlSymsp);
    vlSymsp->TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__VX_Context_one._settle__TOP__Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__VX_Context_one__8(vlSymsp);
    vlTOPp->_settle__TOP__3(vlSymsp);
}

VL_INLINE_OPT QData VVortex::_change_request(VVortex__Syms* __restrict vlSymsp) {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_change_request\n"); );
    VVortex* __restrict vlTOPp VL_ATTR_UNUSED = vlSymsp->TOPp;
    // Body
    // Change detection
    QData __req = false;  // Logically a bool
    return __req;
}

#ifdef VL_DEBUG
void VVortex::_eval_debug_assertions() {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_eval_debug_assertions\n"); );
    // Body
    if (VL_UNLIKELY((clk & 0xfeU))) {
	Verilated::overWidthError("clk");}
    if (VL_UNLIKELY((reset & 0xfeU))) {
	Verilated::overWidthError("reset");}
}
#endif // VL_DEBUG

void VVortex::_ctor_var_reset() {
    VL_DEBUG_IF(VL_DBG_MSGF("+    VVortex::_ctor_var_reset\n"); );
    // Body
    clk = VL_RAND_RESET_I(1);
    reset = VL_RAND_RESET_I(1);
    icache_response_instruction = VL_RAND_RESET_I(32);
    icache_request_pc_address = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    in_cache_driver_out_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    out_cache_driver_in_address[__Vi0] = VL_RAND_RESET_I(32);
    }}
    out_cache_driver_in_mem_read = VL_RAND_RESET_I(3);
    out_cache_driver_in_mem_write = VL_RAND_RESET_I(3);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    out_cache_driver_in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    out_cache_driver_in_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    out_ebreak = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__fetch_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__f_d_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__decode_branch_stall = VL_RAND_RESET_I(1);
    Vortex__DOT__decode_csr_address = VL_RAND_RESET_I(12);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__decode_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__decode_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__decode_itype_immed = VL_RAND_RESET_I(32);
    Vortex__DOT__decode_branch_type = VL_RAND_RESET_I(3);
    Vortex__DOT__decode_jal = VL_RAND_RESET_I(1);
    Vortex__DOT__decode_jal_offset = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__decode_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__decode_change_mask = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__decode_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__d_e_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__d_e_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__d_e_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__execute_branch_stall = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__execute_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__execute_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__execute_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__e_m_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__e_m_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__e_m_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__memory_branch_dir = VL_RAND_RESET_I(1);
    Vortex__DOT__memory_branch_dest = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__memory_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__memory_mem_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__memory_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__m_w_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__m_w_mem_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__m_w_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__writeback_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__csr_decode_csr_data = VL_RAND_RESET_I(32);
    Vortex__DOT__forwarding_fwd_stall = VL_RAND_RESET_I(1);
    Vortex__DOT__forwarding_src1_fwd = VL_RAND_RESET_I(1);
    Vortex__DOT__forwarding_src2_fwd = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__forwarding_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__forwarding_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_fetch__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_fetch__in_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_f_d_reg__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_f_d_reg__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_decode__out_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_decode__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_decode__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_decode__out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_decode__in_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_decode__in_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_decode__in_wb_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_decode__in_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_decode__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_d_e_reg__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_d_e_reg__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_d_e_reg__out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_d_e_reg__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_d_e_reg__in_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_d_e_reg__in_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_execute__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_execute__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_execute__out_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_execute__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_execute__in_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_execute__in_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_e_m_reg__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_e_m_reg__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_e_m_reg__out_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_e_m_reg__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_e_m_reg__in_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_e_m_reg__in_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_memory__out_cache_driver_in_address[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_memory__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_memory__out_mem_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_memory__out_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_memory__in_cache_driver_out_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_memory__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_memory__in_rd2[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_memory__in_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_m_w_reg__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_m_w_reg__out_mem_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_m_w_reg__out_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_m_w_reg__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_m_w_reg__in_mem_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_m_w_reg__in_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_writeback__out_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_writeback__in_mem_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_writeback__in_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_forwarding__out_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellout__vx_forwarding__out_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_mem_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_forwarding__in_writeback_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_forwarding__in_memory_mem_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_forwarding__in_memory_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT____Vcellinp__vx_forwarding__in_execute_alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_fetch__DOT__stall = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_fetch__DOT__warp_num = VL_RAND_RESET_I(4);
    Vortex__DOT__vx_fetch__DOT__warp_state = VL_RAND_RESET_I(4);
    Vortex__DOT__vx_fetch__DOT__warp_count = VL_RAND_RESET_I(4);
    Vortex__DOT__vx_fetch__DOT__add_warp = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_fetch__DOT__remove_warp = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<8; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__warp_glob_pc[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<8; ++__Vi0) {
	    { int __Vi1=0; for (; __Vi1<4; ++__Vi1) {
		    Vortex__DOT__vx_fetch__DOT__warp_glob_valid[__Vi0][__Vi1] = VL_RAND_RESET_I(1);
	    }}
    }}
    Vortex__DOT__vx_fetch__DOT__out_PC_var = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__out_valid_var[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_change_mask = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__warp_zero_stall = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__0__KET____DOT__VX_Warp__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__0__KET____DOT__VX_Warp__in_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_change_mask = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__warp_zero_stall = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Warp__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Warp__in_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_change_mask = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__warp_zero_stall = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Warp__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Warp__in_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_change_mask = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__warp_zero_stall = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Warp__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Warp__in_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_change_mask = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__warp_zero_stall = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Warp__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Warp__in_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_change_mask = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__warp_zero_stall = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Warp__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Warp__in_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_change_mask = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__warp_zero_stall = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Warp__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Warp__in_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_change_mask = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__warp_zero_stall = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Warp__out_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Warp__in_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__real_PC = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__temp_PC = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__0__KET____DOT__VX_Warp__DOT__valid_zero[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__real_PC = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__temp_PC = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__1__KET____DOT__VX_Warp__DOT__valid_zero[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__real_PC = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__temp_PC = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__2__KET____DOT__VX_Warp__DOT__valid_zero[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__real_PC = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__temp_PC = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__3__KET____DOT__VX_Warp__DOT__valid_zero[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__real_PC = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__temp_PC = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__4__KET____DOT__VX_Warp__DOT__valid_zero[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__real_PC = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__temp_PC = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__5__KET____DOT__VX_Warp__DOT__valid_zero[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__real_PC = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__temp_PC = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__6__KET____DOT__VX_Warp__DOT__valid_zero[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__real_PC = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__temp_PC = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_fetch__DOT__genblk1__BRA__7__KET____DOT__VX_Warp__DOT__valid_zero[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_f_d_reg__DOT__instruction = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_f_d_reg__DOT__curr_PC = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_f_d_reg__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_f_d_reg__DOT__warp_num = VL_RAND_RESET_I(4);
    Vortex__DOT__vx_decode__DOT__is_itype = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__is_csr = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__is_clone = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__is_jalrs = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__is_jmprt = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__is_wspawn = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__jal_sys_jal = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__alu_tempp = VL_RAND_RESET_I(12);
    Vortex__DOT__vx_decode__DOT__mul_alu = VL_RAND_RESET_I(5);
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__w0_t0_registers[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<8; ++__Vi0) {
	    { int __Vi1=0; for (; __Vi1<4; ++__Vi1) {
		    Vortex__DOT__vx_decode__DOT__glob_a_reg_data[__Vi0][__Vi1] = VL_RAND_RESET_I(32);
	    }}
    }}
    { int __Vi0=0; for (; __Vi0<8; ++__Vi0) {
	    { int __Vi1=0; for (; __Vi1<4; ++__Vi1) {
		    Vortex__DOT__vx_decode__DOT__glob_b_reg_data[__Vi0][__Vi1] = VL_RAND_RESET_I(32);
	    }}
    }}
    { int __Vi0=0; for (; __Vi0<8; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__glob_clone_stall[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_decode__DOT__real_zero_isclone = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__w0_t0_registers[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__VX_Context_zero__out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__VX_Context_zero__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__temp_out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__temp_out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_decode__DOT__temp_out_clone_stall = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__jalrs_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__jmprt_thread_mask[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_decode__DOT__is_ebreak = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__temp_final_alu = VL_RAND_RESET_I(5);
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__real_wspawn = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__1__KET____DOT__real_isclone = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__1__KET____DOT__VX_Context_one__out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_wspawn_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__1__KET____DOT__VX_Context_one__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__real_wspawn = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__2__KET____DOT__real_isclone = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__2__KET____DOT__VX_Context_one__out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_wspawn_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__2__KET____DOT__VX_Context_one__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__real_wspawn = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__3__KET____DOT__real_isclone = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__3__KET____DOT__VX_Context_one__out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_wspawn_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__3__KET____DOT__VX_Context_one__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__real_wspawn = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__4__KET____DOT__real_isclone = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__4__KET____DOT__VX_Context_one__out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_wspawn_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__4__KET____DOT__VX_Context_one__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__real_wspawn = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__5__KET____DOT__real_isclone = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__5__KET____DOT__VX_Context_one__out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_wspawn_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__5__KET____DOT__VX_Context_one__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__real_wspawn = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__6__KET____DOT__real_isclone = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__6__KET____DOT__VX_Context_one__out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_wspawn_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__6__KET____DOT__VX_Context_one__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__real_wspawn = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_decode__DOT__genblk1__BRA__7__KET____DOT__real_isclone = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellout__genblk1__BRA__7__KET____DOT__VX_Context_one__out_a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_wspawn_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_write_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src2_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_src1_fwd_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT____Vcellinp__genblk1__BRA__7__KET____DOT__VX_Context_one__in_valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__state_stall = VL_RAND_RESET_I(6);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd1_register[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__rd2_register[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__clone_regsiters[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_src2_data = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_src1_data = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__vx_register_file_master__out_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__out_src2_data = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__out_src1_data = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__in_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__out_src2_data = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__out_src1_data = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__in_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__out_src2_data = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellout__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__out_src1_data = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT____Vcellinp__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__in_regs[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__vx_register_file_master__DOT__registers[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__1__KET____DOT__vx_register_file_slave__DOT__registers[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__2__KET____DOT__vx_register_file_slave__DOT__registers[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<32; ++__Vi0) {
	    Vortex__DOT__vx_decode__DOT__VX_Context_zero__DOT__gen_code_label__BRA__3__KET____DOT__vx_register_file_slave__DOT__registers[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_d_e_reg__DOT__rd = VL_RAND_RESET_I(5);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_d_e_reg__DOT__a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_d_e_reg__DOT__b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_d_e_reg__DOT__alu_op = VL_RAND_RESET_I(5);
    Vortex__DOT__vx_d_e_reg__DOT__wb = VL_RAND_RESET_I(2);
    Vortex__DOT__vx_d_e_reg__DOT__PC_next_out = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_d_e_reg__DOT__rs2_src = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_d_e_reg__DOT__itype_immed = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_d_e_reg__DOT__mem_read = VL_RAND_RESET_I(3);
    Vortex__DOT__vx_d_e_reg__DOT__mem_write = VL_RAND_RESET_I(3);
    Vortex__DOT__vx_d_e_reg__DOT__branch_type = VL_RAND_RESET_I(3);
    Vortex__DOT__vx_d_e_reg__DOT__upper_immed = VL_RAND_RESET_I(20);
    Vortex__DOT__vx_d_e_reg__DOT__csr_address = VL_RAND_RESET_I(12);
    Vortex__DOT__vx_d_e_reg__DOT__is_csr = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_d_e_reg__DOT__csr_mask = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_d_e_reg__DOT__curr_PC = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_d_e_reg__DOT__jal = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_d_e_reg__DOT__jal_offset = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_d_e_reg__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_d_e_reg__DOT__reg_data_z[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_d_e_reg__DOT__valid_z[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_d_e_reg__DOT__warp_num = VL_RAND_RESET_I(4);
    Vortex__DOT__vx_d_e_reg__DOT__stalling = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__0__KET____DOT__vx_alu__out_alu_result = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__1__KET____DOT__vx_alu__out_alu_result = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__2__KET____DOT__vx_alu__out_alu_result = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_execute__DOT____Vcellout__genblk1__BRA__3__KET____DOT__vx_alu__out_alu_result = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__ALU_in2 = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_execute__DOT__genblk1__BRA__0__KET____DOT__vx_alu__DOT__mult_signed_result = VL_RAND_RESET_Q(64);
    Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__ALU_in2 = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_execute__DOT__genblk1__BRA__1__KET____DOT__vx_alu__DOT__mult_signed_result = VL_RAND_RESET_Q(64);
    Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__ALU_in2 = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_execute__DOT__genblk1__BRA__2__KET____DOT__vx_alu__DOT__mult_signed_result = VL_RAND_RESET_Q(64);
    Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__ALU_in2 = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_execute__DOT__genblk1__BRA__3__KET____DOT__vx_alu__DOT__mult_signed_result = VL_RAND_RESET_Q(64);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_e_m_reg__DOT__alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_e_m_reg__DOT__rd = VL_RAND_RESET_I(5);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_e_m_reg__DOT__a_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_e_m_reg__DOT__b_reg_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_e_m_reg__DOT__wb = VL_RAND_RESET_I(2);
    Vortex__DOT__vx_e_m_reg__DOT__PC_next = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_e_m_reg__DOT__mem_read = VL_RAND_RESET_I(3);
    Vortex__DOT__vx_e_m_reg__DOT__mem_write = VL_RAND_RESET_I(3);
    Vortex__DOT__vx_e_m_reg__DOT__csr_address = VL_RAND_RESET_I(12);
    Vortex__DOT__vx_e_m_reg__DOT__is_csr = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_e_m_reg__DOT__csr_result = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_e_m_reg__DOT__curr_PC = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_e_m_reg__DOT__branch_offset = VL_RAND_RESET_I(32);
    Vortex__DOT__vx_e_m_reg__DOT__branch_type = VL_RAND_RESET_I(3);
    Vortex__DOT__vx_e_m_reg__DOT__jal = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_e_m_reg__DOT__jal_dest = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_e_m_reg__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_e_m_reg__DOT__warp_num = VL_RAND_RESET_I(4);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_m_w_reg__DOT__alu_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_m_w_reg__DOT__mem_result[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_m_w_reg__DOT__rd = VL_RAND_RESET_I(5);
    Vortex__DOT__vx_m_w_reg__DOT__wb = VL_RAND_RESET_I(2);
    Vortex__DOT__vx_m_w_reg__DOT__PC_next = VL_RAND_RESET_I(32);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_m_w_reg__DOT__valid[__Vi0] = VL_RAND_RESET_I(1);
    }}
    Vortex__DOT__vx_m_w_reg__DOT__warp_num = VL_RAND_RESET_I(4);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_writeback__DOT__out_pc_data[__Vi0] = VL_RAND_RESET_I(32);
    }}
    Vortex__DOT__vx_forwarding__DOT__src1_exe_fwd = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_forwarding__DOT__src1_mem_fwd = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_forwarding__DOT__src1_wb_fwd = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_forwarding__DOT__src2_exe_fwd = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_forwarding__DOT__src2_mem_fwd = VL_RAND_RESET_I(1);
    Vortex__DOT__vx_forwarding__DOT__src2_wb_fwd = VL_RAND_RESET_I(1);
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_forwarding__DOT__use_execute_PC_next[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_forwarding__DOT__use_memory_PC_next[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<4; ++__Vi0) {
	    Vortex__DOT__vx_forwarding__DOT__use_writeback_PC_next[__Vi0] = VL_RAND_RESET_I(32);
    }}
    { int __Vi0=0; for (; __Vi0<1025; ++__Vi0) {
	    Vortex__DOT__vx_csr_handler__DOT__csr[__Vi0] = VL_RAND_RESET_I(12);
    }}
    Vortex__DOT__vx_csr_handler__DOT__cycle = VL_RAND_RESET_Q(64);
    Vortex__DOT__vx_csr_handler__DOT__instret = VL_RAND_RESET_Q(64);
    Vortex__DOT__vx_csr_handler__DOT__decode_csr_address = VL_RAND_RESET_I(12);
    Vortex__DOT__vx_csr_handler__DOT____Vlvbound1 = VL_RAND_RESET_I(12);
    __Vtableidx1 = VL_RAND_RESET_I(3);
    __Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu[0] = 0x10U;
    __Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu[1] = 0x11U;
    __Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu[2] = 0x12U;
    __Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu[3] = 0x13U;
    __Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu[4] = 0x14U;
    __Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu[5] = 0x15U;
    __Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu[6] = 0x16U;
    __Vtable1_Vortex__DOT__vx_decode__DOT__mul_alu[7] = 0x17U;
}
