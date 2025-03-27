/*==============================================================================
C++ Module that provides the definitions and implementation for an event
handler and notification model.

This file contains the following class types
DELEGATES are classes that hold information to indirectly execute the code for a
global function or an instances member function.
The code makes use of C++ templates to effectively create an implementation for
each type of class or function the compiler encounters.
Delegates are then used by EVENTS and CALLBACKS to provide the method to execute
the code
In addition to the function call, delegates hold a 'context' pointer - this can
be optionally specified when creating the delegate (or setting the handler in the
callback or event), and is useful for passing back to the handler a procedure
a pointer to another object or struct that can contain additional processing
information - especially useful for obtaining object pointers when calling
C-style Function delegates.

CALLBACKS are a class that provides a one-to-one caller model for executing a
delegate (unlike Events that provide one-to-many). As no list of Delegates is held,
the memory footprint is smaller than an Event and also faster to execute.

EVENTS are a class that function like a callback, but contain a list of Delegate
handlers that are executed (in order of addition to the list) when the Call function
is raised.
Unlike callbacks, event delegates can't have a return type.

--------------------------------------------------------------------------------
Background...
Based on code from a web articles at ...
http://www.codeproject.com/Articles/6614/Generic-Observer-Pattern-and-Events-in-C?msg=877439
http://www.codeproject.com/Articles/8262/Implementation-of-Delegates-in-C-using-Signal-and
describing "Generic Observer Pattern and events in C++" by Eduard Baranovsky

In Object Oriented Programming an 'Events and Delegates' concept is one of the
most frequently used concepts in programming, sometimes referred to as “Observer”
or 'Document/View' design pattern. Classical formulation of it could be found
in 'Design Patterns, Elements of Reusable Object Oriented Software' by
Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides (The Gang of Four).

This concept is used when you want some information stored in one object, 
called 'model' (subject) to be watched by others, called 'views' (observers).
Each time when information is changed in the 'model', 'views' attached to the
model should receive notification and update there states accordingly to the changed 'model'.

16/09/2013 - Created V1.0 of file, RSh (based on the above)
28/07/2017 - Created V2.0 of file with completly new Event handler class
==============================================================================*/
//Prevent multiple inclusions of this file
#ifndef EVENTS_HPP
#define EVENTS_HPP

//Include common type definitions and macros
#include "common.h"

//Include the Vector (list) library
#include <vector>

//==============================================================================
//Declaration...
//==============================================================================

//==============================================================================
//Zero Parameter Classes
//==============================================================================
/*!-----------------------------------------------------------------------------
Declare an abstract base class from which all EventHandlers derive, allowing us
to attach different types of event handles (member functions with same types of 
parameters from different classes) to the same event. 
We use templates to make it generic for any combination of parameter types in 
"delegate" or call back method. There are different event types for every number
of arguments in callback function.
*/
template <typename ReturnT>
class CDelegate0
{
	public:		
        virtual ~CDelegate0() {}
		virtual ReturnT Call() = 0;		
};

/*!-----------------------------------------------------------------------------
Delegate a class that uses templates to create an implementation for each type
of handler needed for methods in different derived Listener classes
The 'context' parameter allows a generic pointer to be stored that may be used to
represent a strucutre, or class instance that is passed back to the handler
procedure each time the delegate is called (especially useful when calling C-Style
functions for DLL's etc).
*/
template <typename ListenerT, typename ReturnT>
class CDelegateMethod0 : public CDelegate0<ReturnT>
{	
	private:
        typedef ReturnT (ListenerT::*PMember)(pointer);
		ListenerT* _instance;
		PMember _member;
        pointer _context;

	public:
		//Constructor
        CDelegateMethod0(ListenerT* instance, PMember member, pointer context = NULL) : _instance(instance), _member(member), _context(context) { }

		//Notification function
        ReturnT Call() { return (_instance->*_member)(_context); }
};

/*!-----------------------------------------------------------------------------
Delegate a class that uses templates to create an implementation for each type
of handler needed for static/global functions (not in classes)
The 'context' parameter allows a generic pointer to be stored that may be used to
represent a strucutre, or class instance that is passed back to the handler
procedure each time the delegate is called (especially useful when calling C-Style
functions for DLL's etc).
*/
template <typename ReturnT>
class CDelegateFunction0 : public CDelegate0<ReturnT>
{	
	private:
        typedef ReturnT (*PFunction)(pointer);
		PFunction _function;
        pointer _context;

	public:
		//Constructor
        CDelegateFunction0(PFunction func, pointer context = NULL) : _function(func), _context(context) { }

		//Notification function
        ReturnT Call() { return (*_function)(_context); }
};

/*!-----------------------------------------------------------------------------
Class that stores a pointer to a delegate object, and executes it when the Call
method is called
This class is coded to accept no delegate parameters
*/
template <typename ReturnT>
class CCallback0 {
	private :		
		typedef CDelegate0<ReturnT>* PDelegate;
		PDelegate _delegate;
		
	public :
		//Constructor
		CCallback0() { _delegate = NULL; }
		
		//Destructor
		~CCallback0() { this->Clear(); }
		
		//Function called to execute the callback and return its value (or the default value of the return type if not defined)
		ReturnT Call() { if(_delegate) return _delegate->Call(); else return ReturnT();}
		
		//Clear Function
		inline void Clear() { if(_delegate) { delete _delegate; } }
		
		//Set the Callback to execute an instances member function
		template <typename ListenerT>
        void Set(ListenerT* instance, ReturnT (ListenerT::*member)(pointer), pointer context = NULL)
		{
			this->Clear();
            _delegate = new CDelegateMethod0<ListenerT, ReturnT>(instance, member, context);
		}
		
		//Sets the Callback to execute a static or global function
        void Set(ReturnT (*func)(pointer), pointer context = NULL)
		{
			this->Clear();
            _delegate = new CDelegateFunction0<ReturnT>(func, context);
		}			
};

/*!----------------------------------------------------------------------------
Class that stores a list of delegate objects, and executes each one when the
Call method is called (whereas a 'callback' class only works with one delegate)
This class is coded to only accept one-parameter delegates
*/
class CEvent0
{
    private:
        typedef CDelegate0<void> CEventDelegate;   //Event delegates can't have a return type - hence 'void' */
        typedef CEventDelegate* PEventDelegate;
        typedef std::vector<PEventDelegate> TDelegateList;
        TDelegateList _delegates;

    public:
        /*!---------------------------------------------------------------------
        Constructor
        */
        CEvent0()
        {
            _delegates.clear();
        }

        /*!---------------------------------------------------------------------
        Destructor for the event class
        */
        virtual ~CEvent0()
        {
            this->Clear();
        }

        /*!---------------------------------------------------------------------
        Function that attaches a handler for an objects member function to the event
        @result Pointer to the created delegate object - store and use this pointer to detach the delegate from the event if needed
        */
        template <typename ListenerT>
        pointer AttachMethod(ListenerT* instance, void (ListenerT::*member)(pointer), pointer context = NULL)
        {
            //Create and store a new delegate object for the specified Listner object and its member function
            PEventDelegate delegate = new CDelegateMethod0<ListenerT, void>(instance, member, context);
            _delegates.push_back(delegate);
            return (pointer)delegate;
        }

        /*!---------------------------------------------------------------------
        Function that attaches a handler for a static or global (C type) function to the event
        @result Pointer to the created delegate object - store and use this pointer to detach the delegate from the event if needed
        */
        pointer AttachFunc(void (*func)(pointer), pointer context = NULL)
        {
            //Create and store a new delegate object for the specified function
            PEventDelegate delegate = new CDelegateFunction0<void>(func, context);
            _delegates.push_back(delegate);
            return (pointer)delegate;
        }

        /*!---------------------------------------------------------------------
        Function that clears out all attached delegate handlers
        */
        void Clear() {
            for(uint32 idx = 0; idx < _delegates.size(); idx++) {
                delete _delegates[idx];
            }
            _delegates.clear();
        }

        /*!---------------------------------------------------------------------
        Function that detaches a delegate from the event. Use the pointer passed
        @param delegate is the pointer (type void*) returned from the Attach functions that specifies the delegate to remove
        @result true if the delegate was found and removed.
        */
        bool Detach(pointer delegate)
        {
            for(uint32 idx = 0; idx < _delegates.size(); idx++) {
                if(delegate == (pointer)_delegates[idx]) {
                    delete _delegates[idx];
                    _delegates.erase(_delegates.begin() + idx);
                    return true;
                }
            }
            return false;
        }

        /*!---------------------------------------------------------------------
        Function called to "notify" all attached Listner object of the event, by
        executing their specified notification method
        */
        void Call()
        {
            for(uint32 idx = 0; idx < _delegates.size(); idx++) {
                //Execute the notification method of the listner object (the value in the iterator key/value pair)
                _delegates[idx]->Call();
            }
        }
};

//==============================================================================
//One Parameter Classes
//==============================================================================
/*!-----------------------------------------------------------------------------
Declare an abstract base class from which all EventHandlers derive, allowing us
to attach different types of event handles (member functions with same types of 
parameters from different classes) to the same event. 
We use templates to make it generic for any combination of parameter types in 
'delegate' or call back method. There are different event types for every number
of arguments in callback function.
*/
template <typename ReturnT, typename Param1T>
class CDelegate1
{    
	public:	
        virtual ~CDelegate1() {}
		virtual ReturnT Call(Param1T param) = 0;
};

/*!-----------------------------------------------------------------------------
Delegate a class that uses templates to create an implementation for each type
of handler needed for methods in different derived Listener classes
The 'context' parameter allows a generic pointer to be stored that may be used to
represent a strucutre, or class instance that is passed back to the handler
procedure each time the delegate is called (especially useful when calling C-Style
functions for DLL's etc).
*/
template <typename ListenerT, typename ReturnT, typename Param1T>
class CDelegateMethod1 : public CDelegate1<ReturnT, Param1T>
{	
	private:	
        typedef ReturnT (ListenerT::*PMember)(pointer, Param1T);
		ListenerT* _instance;
		PMember _member;		
        pointer _context;

	public:
		//Constructor
        CDelegateMethod1(ListenerT* instance, PMember member, pointer context = NULL) : _instance(instance), _member(member), _context(context) { }

		//Notification function
        ReturnT Call(Param1T param) { return (_instance->*_member)(_context, param); }
};

/*!-----------------------------------------------------------------------------
Delegate a class that uses templates to create an implementation for each type
of handler needed for static/global functions (not in classes)
The 'context' parameter allows a generic pointer to be stored that may be used to
represent a strucutre, or class instance that is passed back to the handler
procedure each time the delegate is called (especially useful when calling C-Style
functions for DLL's etc).
*/
template <typename ReturnT, typename Param1T>
class CDelegateFunction1 : public CDelegate1<ReturnT, Param1T>
{	
	private:
        typedef ReturnT (*PFunction)(pointer, Param1T);
		PFunction _function;		
        pointer _context;

	public:
		//Constructor
		CDelegateFunction1(PFunction func, pointer context = NULL) : _function(func), _context(context) { }

		//Notification function
        ReturnT Call(Param1T param) { return (*_function)(_context, param); }
};


/*!-----------------------------------------------------------------------------
Class that stores a pointer to a delegate object, and executes it when the Call
method is called
This class is coded to accept no delegate parameters
*/
template <typename ReturnT, typename Param1T>
class CCallback1 {
	private :		
		typedef CDelegate1<ReturnT, Param1T>* PDelegate;
		PDelegate _delegate;
		
	public :
		//Constructor
		CCallback1() { _delegate = NULL; }
		
		//Destructor
		~CCallback1() { this->Clear(); }
		
		//Function called to execute the callback and return its value (or the default value of the return type if not defined)
		ReturnT Call(Param1T param1) { if(_delegate) return _delegate->Call(param1); else return ReturnT();}
		
		//Clear Function
		inline void Clear() { if(_delegate) { delete _delegate; } }
		
		//Set the Callback to execute an instances member function
		template <typename ListenerT>
        void SetMethod(ListenerT* instance, ReturnT (ListenerT::*member)(pointer, Param1T), pointer context = NULL)
		{
			this->Clear();
			_delegate = new CDelegateMethod1<ListenerT, ReturnT, Param1T>(instance, member, context); 			
		}
		
		//Sets the Callback to execute a static or global function
        void SetFunc(ReturnT (*func)(pointer, Param1T), pointer context = NULL)
		{
			this->Clear();
			_delegate = new CDelegateFunction1<ReturnT, Param1T>(func, context);			
		}			
};

/*!----------------------------------------------------------------------------
Class that stores a list of delegate objects, and executes each one when the
Call method is called (whereas a 'callback' class only works with one delegate)
This class is coded to only accept one-parameter delegates
*/
template <typename Param1T>
class CEvent1
{
    private:
        typedef CDelegate1<void, Param1T> CEventDelegate;   //Event delegates can't have a return type - hence 'void' */
        typedef CEventDelegate* PEventDelegate;
        typedef std::vector<PEventDelegate> TDelegateList;
        TDelegateList _delegates;

    public:
        /*!---------------------------------------------------------------------
        Constructor
        */
        CEvent1()
        {
            _delegates.clear();
        }

        /*!---------------------------------------------------------------------
        Destructor for the event class
        */
        virtual ~CEvent1()
        {
            this->Clear();
        }

        /*!---------------------------------------------------------------------
        Function that attaches a handler for an objects member function to the event
        @result Pointer to the created delegate object - store and use this pointer to detach the delegate from the event if needed
        */
        template <typename ListenerT>
        pointer AttachMethod(ListenerT* instance, void (ListenerT::*member)(pointer, Param1T), pointer context = NULL)
        {
            //Create and store a new delegate object for the specified Listner object and its member function
            PEventDelegate delegate = new CDelegateMethod1<ListenerT, void, Param1T>(instance, member, context);
            _delegates.push_back(delegate);
            return (pointer)delegate;
        }

        /*!---------------------------------------------------------------------
        Function that attaches a handler for a static or global (C type) function to the event
        @result Pointer to the created delegate object - store and use this pointer to detach the delegate from the event if needed
        */
        pointer AttachFunc(void (*func)(pointer, Param1T), pointer context = NULL)
        {
            //Create and store a new delegate object for the specified function
            PEventDelegate delegate = new CDelegateFunction1<void, Param1T>(func, context);
            _delegates.push_back(delegate);
            return (pointer)delegate;
        }

        /*!---------------------------------------------------------------------
        Function that clears out all attached delegate handlers
        */
        void Clear() {
            for(uint32 idx = 0; idx < _delegates.size(); idx++) {
                delete _delegates[idx];
            }
            _delegates.clear();
        }

        /*!---------------------------------------------------------------------
        Function that detaches a delegate from the event. Use the pointer passed
        @param delegate is the pointer (type void*) returned from the Attach functions that specifies the delegate to remove
        @result true if the delegate was found and removed.
        */
        bool Detach(pointer delegate)
        {
            for(uint32 idx = 0; idx < _delegates.size(); idx++) {
                if(delegate == (pointer)_delegates[idx]) {
                    delete _delegates[idx];
                    _delegates.erase(_delegates.begin() + idx);
                    return true;
                }
            }
            return false;
        }

        /*!---------------------------------------------------------------------
        Function called to "notify" all attached Listner object of the event, by
        executing their specified notification method
        */
        void Call(Param1T param)
        {
            for(uint32 idx = 0; idx < _delegates.size(); idx++) {
                //Execute the notification method of the listner object (the value in the iterator key/value pair)
                _delegates[idx]->Call(param);
            }
        }
};

//==============================================================================
//Common Callback Type Definitions
//==============================================================================
/*! Define a simple callback for functions that take no parameters, and return nothing
typedef CCallback0<void> CCallback;
*/

/*! Define a pointer to a simple callback
typedef CCallback* PCallback;
*/

//------------------------------------------------------------------------------
/*! Define a standard single parameter Notification event - the first param may hold a pointer to the sender object
struct TNotifyCallbackParams {
	pointer Sender;
};
*/

/*! Define a simple notification event, that can pass the sender
typedef CCallback1<void, TNotifyCallbackParams&> CNotifyCallback;
*/

//==============================================================================
#endif
