SQLITE ?= y

ifeq ($(SQLITE),y)

$(eval $(call pkg-config-library,LIBSQLITE,sqlite3))

SQLITE_CPPFLAGS = $(LIBSQLITE_CPPFLAGS)
SQLITE_LDLIBS = $(LIBSQLITE_LDLIBS)

endif
