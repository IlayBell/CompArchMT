/* 046267 Computer Architecture - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <stdio.h>
#include <vector>
#include <iostream>



class Thread {
	int threadid;
	tcontext* context; // reg file
	std::vector<Instruction*> instructions;
	bool halt;
	int wait_cycles;
	int inst_idx;

	public:
		Thread(int threadid) {
			this->threadid = threadid;
			context = new tcontext();

			this->clean_ctx();

			// Loading the commands to vector
			uint32_t line = 0;
			do {
				Instruction* inst = new Instruction();
				SIM_MemInstRead(line++, inst, this->threadid);
				instructions.push_back(inst);
			} while (instructions.back()->opcode == CMD_HALT);
			
			this->halt = false;
			this->wait_cycles = 0;
			this->inst_idx = 0;

		}

		tcontext* get_context_p() {
			return this->context;
		}

		void clean_ctx() {
			for (int i = 0; i < REGS_COUNT; i++) {
				this->context->reg[i] = 0;
			}
		}

		void set_wait_cycles(int cycles) {
			this->wait_cycles = cycles;
		}

		void update_wait_cycles(int cycles) {
			this->wait_cycles -= cycles;

			if (this->wait_cycles < 0) {
				this->wait_cycles = 0;
			}
		}

		int get_wait_cycles() {
			return this->wait_cycles; 
		}

		void set_halt() {
			this->halt = true;
		}

		bool get_halt() {
			return this->halt;
		}

		Instruction* get_inst_thread() {
			Instruction* inst = this->instructions.at(this->inst_idx);
			this->inst_idx++;
			return inst;
		}

		~Thread() {
			delete this->context;

			// Fail safe
			while (!this->instructions.empty()) {
				Instruction* inst = this->instructions.front();
				this->instructions.erase(this->instructions.begin());
				delete inst;
			}
		}
};

// Global Vector of threads
std::vector<Thread*> threads_blocked;
std::vector<Thread*> threads_fg;

int inst_num_blocked = 0;
int cycles_blocked = 0;

int inst_num_fg = 0;
int cycles_fg = 0;

/**
 * @name context_switch
 * @brief Performs context switch using RR method
 * @param[in] threads - The desired thread vector.
 * @param[in] curr_t - Current thread idx.
 * @param[out] target_t - Next thread to run. If can't do, return curr_t.
 * @return if context switched 
 */
bool context_switch(std::vector<Thread*> threads, int curr_t, int* target_t) {
	int next_thread_idx = curr_t;
	for (int i = 1; i < SIM_GetThreadsNum(); i++) {
		// Cyclic run over threads
		next_thread_idx = (curr_t + i) % SIM_GetThreadsNum();
		Thread* next_thread = threads.at(next_thread_idx);

		if (!next_thread->get_halt() && next_thread->get_wait_cycles() == 0) {
			*target_t = next_thread_idx;
			return true;
		}
	}

	*target_t = curr_t;
	return false;
}


bool check_done_exec(std::vector<Thread*> threads) {
	for(Thread* thread : threads) {
		if (!thread->get_halt()) {
			return false;
		}
	}

	return true;
}

void CORE_BlockedMT() {
	// Init all threads

	threads_blocked.clear();
	for (int i = 0; i < SIM_GetThreadsNum(); i++) {
		Thread* thread = new Thread(i);
		threads_blocked.push_back(thread);
	}

	int thread_num = 0;

	bool ctx_switch_flag = false;
	int next_thread = 0;

	while (!check_done_exec(threads_blocked)) {
		Thread* thread = threads_blocked.at(thread_num);

		Instruction* inst = thread->get_inst_thread();
		inst_num_blocked++;

		int* reg_file = thread->get_context_p()->reg;

		// Cycles maintenance for threads
		cycles_blocked++;
		for (Thread* t : threads_blocked) {
			t->update_wait_cycles(1);
		}

		switch (inst->opcode) {
			case CMD_ADD: // dst <- src1 + src2
				reg_file[inst->dst_index] = reg_file[inst->src1_index] + reg_file[inst->src2_index_imm];
				break;
			case CMD_SUB: // dst <- src1 - src2
				reg_file[inst->dst_index] = reg_file[inst->src1_index] - reg_file[inst->src2_index_imm];
				break;
			case CMD_ADDI: // dst <- src1 + imm
				reg_file[inst->dst_index] = reg_file[inst->src1_index] + inst->src2_index_imm;
				break;
			case CMD_SUBI: // dst <- src1 - imm
				reg_file[inst->dst_index] = reg_file[inst->src1_index] - inst->src2_index_imm;
				break;
			case CMD_LOAD: //dst <- Mem[src1 + src2]  (src2 may be an immediate)
				// ASSUMES NO DEPENDENCIES BETWEEN THREADS

				// Computing address in case of src2 being imm or not.
				uint32_t addr = reg_file[inst->src1_index] + (inst->isSrc2Imm ? inst->src2_index_imm : reg_file[inst->src2_index_imm]);
				SIM_MemDataRead(addr, reg_file + inst->dst_index); // Used pointer arithmetics

				thread->set_wait_cycles(SIM_GetLoadLat());

				ctx_switch_flag = context_switch(threads_blocked, thread_num, &next_thread);

				// ctx switch has penalty in blocked
				cycles_blocked += SIM_GetSwitchCycles(); 
				for (Thread* t : threads_blocked) {
					t->update_wait_cycles(SIM_GetSwitchCycles());
				}

				if (!ctx_switch_flag) {
					while(thread->get_wait_cycles()) {
						cycles_blocked++;
						for (Thread* t : threads_blocked) {
							t->update_wait_cycles(1);
						}
					}
				}

				thread_num = next_thread;

				break;
			case CMD_STORE: // Mem[dst + src2] <- src1  (src2 may be an immediate)
				// ASSUMES NO DEPENDENCIES BETWEEN THREADS

				// Computing address in case of src2 being imm or not.
				uint32_t addr = reg_file[inst->dst_index] + (inst->isSrc2Imm ? inst->src2_index_imm : reg_file[inst->src2_index_imm]);
				SIM_MemDataWrite(addr, reg_file[inst->src1_index]); // Used pointer arithmetics

				thread->set_wait_cycles(SIM_GetStoreLat());
				ctx_switch_flag = context_switch(threads_blocked, thread_num, &next_thread);
				
				// ctx switch has penalty in blocked
				cycles_blocked += SIM_GetSwitchCycles(); 
				for (Thread* t : threads_blocked) {
					t->update_wait_cycles(SIM_GetSwitchCycles());
				}

				if (!ctx_switch_flag) {
					while(thread->get_wait_cycles()) {
						cycles_blocked++;
						for (Thread* t : threads_blocked) {
							t->update_wait_cycles(1);
						}
					}
				}

				thread_num = next_thread;

				break;
			
			case CMD_HALT:
				thread->set_halt();

				ctx_switch_flag = context_switch(threads_blocked, thread_num, &next_thread);

				if (ctx_switch_flag) {
					thread_num = next_thread;
				} else {
					// Wait until there is another available thread.
					while(!ctx_switch_flag) {
						cycles_blocked++;
						for (Thread* t : threads_blocked) {
							t->update_wait_cycles(1);
						}

						ctx_switch_flag = context_switch(threads_blocked, thread_num, &next_thread);
					}

					thread_num = next_thread;
				}

				cycles_blocked += SIM_GetSwitchCycles(); // ctx switch has penalty in blocked
				break;

			default: // NOP
				break;
		}
	}
}

void CORE_FinegrainedMT() {
	// Init all threads

	threads_fg.clear();
	for (int i = 0; i < SIM_GetThreadsNum(); i++) {
		Thread* thread = new Thread(i);
		threads_fg.push_back(thread);
	}

	int thread_num = 0;

	bool ctx_switch_flag = false;
	int next_thread = 0;

	while (!check_done_exec(threads_fg)) {
		Thread* thread = threads_fg.at(thread_num);

		Instruction* inst = thread->get_inst_thread();
		inst_num_fg++;

		int* reg_file = thread->get_context_p()->reg;

		// Cycles maintenance for threads
		cycles_fg++;
		for (Thread* t : threads_fg) {
			t->update_wait_cycles(1);
		}

		switch (inst->opcode) {
			case CMD_ADD: // dst <- src1 + src2
				reg_file[inst->dst_index] = reg_file[inst->src1_index] + reg_file[inst->src2_index_imm];
				break;
			case CMD_SUB: // dst <- src1 - src2
				reg_file[inst->dst_index] = reg_file[inst->src1_index] - reg_file[inst->src2_index_imm];
				break;
			case CMD_ADDI: // dst <- src1 + imm
				reg_file[inst->dst_index] = reg_file[inst->src1_index] + inst->src2_index_imm;
				break;
			case CMD_SUBI: // dst <- src1 - imm
				reg_file[inst->dst_index] = reg_file[inst->src1_index] - inst->src2_index_imm;
				break;
			case CMD_LOAD: //dst <- Mem[src1 + src2]  (src2 may be an immediate)
				// ASSUMES NO DEPENDENCIES BETWEEN THREADS

				// Computing address in case of src2 being imm or not.
				uint32_t addr = reg_file[inst->src1_index] + (inst->isSrc2Imm ? inst->src2_index_imm : reg_file[inst->src2_index_imm]);
				SIM_MemDataRead(addr, reg_file + inst->dst_index); // Used pointer arithmetics

				break;
			case CMD_STORE: // Mem[dst + src2] <- src1  (src2 may be an immediate)
				// ASSUMES NO DEPENDENCIES BETWEEN THREADS

				// Computing address in case of src2 being imm or not.
				uint32_t addr = reg_file[inst->dst_index] + (inst->isSrc2Imm ? inst->src2_index_imm : reg_file[inst->src2_index_imm]);
				SIM_MemDataWrite(addr, reg_file[inst->src1_index]); // Used pointer arithmetics

				break;
			
			case CMD_HALT:
				thread->set_halt();
				break;

			default: // NOP
				break;
			
		}

		ctx_switch_flag = context_switch(threads_blocked, thread_num, &next_thread);

		if (!ctx_switch_flag) {
			// Wait until there is another available thread.
			while(!ctx_switch_flag) {
				cycles_blocked++;
				for (Thread* t : threads_blocked) {
					t->update_wait_cycles(1);
				}

				ctx_switch_flag = context_switch(threads_blocked, thread_num, &next_thread);
			}
		}

		thread_num = next_thread;
	}
}

double CORE_BlockedMT_CPI(){
	// Deletes allocated space
	for (int i = 0; i < SIM_GetThreadsNum(); i++) {
		Thread* thread = threads_blocked.front();
		threads_blocked.erase(threads_blocked.begin());
		delete thread;
	}

	if (cycles_blocked == 0) {
		std::cerr << "No cycles were ran in Blocked." << std::endl;
		return  -1;
	}

	return (double)inst_num_blocked / (double)cycles_blocked;
}

double CORE_FinegrainedMT_CPI(){
	// Calculate IPC

	// Deletes allocated space
	for (int i = 0; i < SIM_GetThreadsNum(); i++) {
		Thread* thread = threads_fg.front();
		threads_fg.erase(threads_fg.begin());
		delete thread;
	}

	if (cycles_fg == 0) {
		std::cerr << "No cycles were ran in Fine-Grained." << std::endl;
		return  -1;
	}

	return (double)inst_num_fg / (double)cycles_fg;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) {
	*context = *(threads_blocked.at(threadid)->get_context_p());
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) {
	*context = *(threads_fg.at(threadid)->get_context_p());
}
