#pragma once
#include "StdAfx.h"
inline void WaitTick() { Wait(1); }
class CSemaphore
{
    bool available;
public:
    CSemaphore(LONG initial, LONG) : available(initial != 0) {}
    bool Acquire(int) { bool result = available; available = false; return result; }
    void Release() { available = true; }
};
class CSimpleTask
{
protected:
    volatile BOOL m_Working = false;
};
