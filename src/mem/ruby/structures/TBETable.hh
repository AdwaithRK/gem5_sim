/*
 * Copyright (c) 2020 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_STRUCTURES_TBETABLE_HH__
#define __MEM_RUBY_STRUCTURES_TBETABLE_HH__

#include <iostream>
#include <unordered_map>
#include <bits/stdc++.h>

#include "mem/ruby/common/Address.hh"

static int count =0;
static gem5::Tick penality = 0;



namespace gem5
{

namespace ruby
{


template<class ENTRY>
class TBETable
{
  public:
    TBETable(int number_of_TBEs)
        : m_number_of_TBEs(number_of_TBEs)
    {
    }
    std::unordered_map<Addr , std::pair<Tick, Tick>> penality_buffer_a; 
    bool isPresent(Addr address) const;
    void allocate(Addr address);
    void deallocate(Addr address, int type);
    bool
    areNSlotsAvailable(int n, Tick current_time) const
    {
        return (m_number_of_TBEs - m_map.size()) >= n;
    }

    ENTRY *getNullEntry();
    ENTRY *lookup(Addr address);

    // Print cache contents
    void print(std::ostream& out) const;

  protected:
    // Protected copy constructor and assignment operator
    TBETable(const TBETable& obj);
    TBETable& operator=(const TBETable& obj);
    void incrementpenalty(Tick x);
    Tick get_avg_misspenalty();
    Tick m_sumtotal;
    int m_cnttotal;  
    // Tick m_sumtotal;
    // int m_cnttotal;   

    // Data Members (m_prefix)
    std::unordered_map<Addr, ENTRY> m_map;

  private:
    int m_number_of_TBEs;
};

template<class ENTRY>
inline std::ostream&
operator<<(std::ostream& out, const TBETable<ENTRY>& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

template<class ENTRY>
inline bool
TBETable<ENTRY>::isPresent(Addr address) const
{
    assert(address == makeLineAddress(address));
    assert(m_map.size() <= m_number_of_TBEs);
    return !!m_map.count(address);

}

template<class ENTRY>
inline void
TBETable<ENTRY>::allocate(Addr address)
{
    assert(!isPresent(address));
    assert(m_map.size() < m_number_of_TBEs);
    m_map[address] = ENTRY();

    m_map[address].m_entry_time = curTick();
    m_map[address].m_exit_time = INT64_MAX;
}

template<class ENTRY>
inline void
TBETable<ENTRY>::deallocate(Addr address, int type)
{
    assert(isPresent(address));
    assert(m_map.size() > 0);
    m_map[address].m_exit_time =  curTick();
    if(type == 1){
      std::cout << "here in TBE type 1\n\n";
      incrementpenalty(m_map[address].m_exit_time  - m_map[address].m_entry_time);
      //get_avg_misspenalty();
    }

    m_map.erase(address);
}

template<class ENTRY>
inline ENTRY*
TBETable<ENTRY>::getNullEntry()
{
    return nullptr;
}

template<class ENTRY>
inline void
TBETable<ENTRY>::incrementpenalty(Tick x){
    penality += x;
    count += 1;
    m_sumtotal+=x;
    m_cnttotal+=1;
}

// looks an address up in the cache
template<class ENTRY>
inline ENTRY*
TBETable<ENTRY>::lookup(Addr address)
{
  if (m_map.find(address) != m_map.end()) return &(m_map.find(address)->second);
  return NULL;
}

template<class ENTRY>
inline Tick
TBETable<ENTRY>::get_avg_misspenalty(){


  if(m_sumtotal!=0&&m_cnttotal!=0){
    // if(flag){
    // DPRINTF(MyRuby,"Final miss penalty L1Cache Core %d - Tick = %d, Sum = %d, Cnt = %d, Avg Miss Penalty = %d\n\n", m_coreID,curTick(), m_sumtotal,m_cnttotal, m_sumtotal/m_cnttotal);
    // }

    return m_sumtotal/m_cnttotal;
  }
  else return 1;

}


template<class ENTRY>
inline void
TBETable<ENTRY>::print(std::ostream& out) const
{
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_TBETABLE_HH__