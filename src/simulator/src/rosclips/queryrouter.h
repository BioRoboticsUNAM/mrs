#ifndef __QUERYROUTER_H__
#define __QUERYROUTER_H__

#pragma once

#include <string>
#include "clipswrapper.h"

class QueryRouter{
// Singleton element access
public:
	/**
	 * Returns the unique instance of QueryRouter, creating it if necessary
	 * @param  routerName The name to be assigned to the router
	 * @param  priority   The priority to be assigned to the router
	 * @return            A unique router (singleton)
	 */
	static QueryRouter& getInstance(
		const std::string& routerName = "query",
		clips::RouterPriority priority = clips::RouterPriority::StandardShared);

// Disable copy constructor and copy assignation
	QueryRouter(const QueryRouter&)    = delete;
	void operator=(const QueryRouter&) = delete;

// Make constructor private for Singleton
private:
	QueryRouter(const std::string& routerName,
		clips::RouterPriority priority);



public:
	~QueryRouter();

public:
	/**
	 * Enables the router, storing output data in the buffer
	 */
	void enable();
	/**
	 * Disables the router. Data won't be stored in the buffer
	 */
	void disable();

	/**
	 * Gets a value indicating if the router is enabled
	 * @return true if the router is enabled, false otherwise
	 */
	bool isEnabled();

	/**
	 * Returns the router's name
	 */
	std::string getName();

	/**
	 * Returns the priority of the router
	 */
	clips::RouterPriority getPriority();

	/**
	 * Retrieves the OR-ed set of logical names captured by this router
	 */
	clips::LogicalName getLogicalNames();

	/**
	 * Sets the OR-ed set of logical names to be captured by this router
	 */
	void setLogicalNames(const clips::LogicalName& flags);

	/**
	 * Returns the data in the buffer.
	 * The buffer is cleared afterwards.
	 * @return The string contained in the internal buffer
	 */
	std::string read();

	/**
	 * Writes into the router internal buffer (append)
	 * @param s Data to be appended into the buffer
	 */
	void write(const std::string& s);

private:
	/**
	 * Registers the router with CLIPS
	 */
	void registerR();
	/**
	 * Unregisters the router from CLIPS
	 */
	void unregisterR();

private:
	bool enabled;
	bool registered;
	std::string routerName;
	clips::LogicalName lnFlags;
	clips::RouterPriority priority;
	std::string buffer;
};

#endif // __QUERYROUTER_H__