CC = gcc
CFLAGS = -Wall -Wextra -I$(SRCDIR)
LDLIBS = -lm

SRCDIR = src
OBJDIR = obj
BINDIR = exec
LOGDIR = logs

COMMON_OBJS = $(OBJDIR)/log.o

TARGETS = main drone obstacle blackboard input target watchdog

all: setup $(TARGETS)

# =================== OGGETTI ===================
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

# =================== LINK ===================
main: $(OBJDIR)/main.o $(COMMON_OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $^ -o $(BINDIR)/$@ -lncurses

drone: $(OBJDIR)/drone.o $(COMMON_OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $^ -o $(BINDIR)/$@ $(LDLIBS)

obstacle: $(OBJDIR)/obstacle.o $(COMMON_OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $^ -o $(BINDIR)/$@ $(LDLIBS)

blackboard: $(OBJDIR)/blackboard.o $(COMMON_OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $^ -o $(BINDIR)/$@ -lncursesw

target: $(OBJDIR)/target.o $(COMMON_OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $^ -o $(BINDIR)/$@ $(LDLIBS)

input: $(OBJDIR)/input.o $(COMMON_OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $^ -o $(BINDIR)/$@ -lncurses

watchdog: $(OBJDIR)/watchdog.o $(COMMON_OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $^ -o $(BINDIR)/$@

# =================== UTILS ===================
setup:
	@mkdir -p $(OBJDIR) $(BINDIR) $(LOGDIR)
	@touch $(LOGDIR)/system.log $(LOGDIR)/watchdog.log

clean:
	rm -rf $(OBJDIR)

cleanall: clean
	rm -rf $(BINDIR) $(LOGDIR)

run: all
	@rm -f $(LOGDIR)/*.log
	@mkdir -p $(LOGDIR)
	@touch $(LOGDIR)/system.log $(LOGDIR)/watchdog.log
	./$(BINDIR)/main
