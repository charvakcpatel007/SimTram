/*   
    Copyright (C) 2011 ApPeAL Group, Politecnico di Torino

    This file is part of TraCI4J.

    TraCI4J is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    TraCI4J is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with TraCI4J.  If not, see <http://www.gnu.org/licenses/>.
*/

package it.polito.appeal.traci;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.ConnectException;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketAddress;
import java.util.ArrayList;
import java.util.List;

import de.tudresden.sumo.cmd.Vehicle;
import de.tudresden.sumo.config.Constants;
import de.tudresden.sumo.util.CommandProcessor;
import de.tudresden.sumo.util.SumoCommand;

/**
 * Models a TCP/IP connection to a local or remote SUMO server via the TraCI
 * protocol.
 * 
 * @author Enrico Gueli &lt;enrico.gueli@gmail.com&gt;
 * @author Mario Krumnow
 * 
 */
public class SumoTraciConnection {

	/**
	 * Reads an InputStream object and logs each row in the containing class's
	 * logger.
	 * 
	 * @author Enrico
	 * 
	 */
	private static class StreamLogger implements Runnable {
		final InputStream stream;
		@SuppressWarnings("unused")
		final String prefix;

		public StreamLogger(InputStream stream, String prefix) {
			this.stream = stream;
			this.prefix = prefix;
		}

		public void run() {
		
			BufferedReader br = new BufferedReader(new InputStreamReader(stream));
			try {
				
				String strLine;
				while ((strLine = br.readLine()) != null)   {
					
					if(strLine.contains("Error:") && !strLine.contains("peer shutdown")){System.err.println (strLine);}
					//else{System.out.println (strLine);}
					
				  }	
				
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	private String configFile;
	private int randomSeed;
	private int remotePort;
	private Socket socket;
	
	//new
	private String net_file;
	private String route_file;
	String sumoEXE = "/opt/sumo/sumo-0.15.0/bin/sumo";
	private CommandProcessor cp;
	
	private Process sumoProcess;
	private static final int CONNECT_RETRIES = 3;
	@SuppressWarnings("unused")
	private CloseQuery closeQuery;
	private List<String> args = new ArrayList<String>();
	
	private boolean remote = false;
	
	public SumoTraciConnection(String sumo_bin, String net_file, String route_file) {
		this.sumoEXE=sumo_bin;
		this.net_file=net_file;
		this.route_file=route_file;
	}
	
	public SumoTraciConnection(String sumo_bin, String configFile) {
		this.sumoEXE=sumo_bin;
		this.configFile=configFile;
	}
	
	
	
	public SumoTraciConnection(String configFile, int randomSeed, boolean useGeoOffset) {
		this.randomSeed = randomSeed;
		this.configFile = configFile;
	}

	
	public SumoTraciConnection(SocketAddress sockAddr) throws IOException,
			InterruptedException {
		
		this.remote=true;
		socket = new Socket();
		socket.setTcpNoDelay(true);
		
		int waitTime = 500; // milliseconds
		for (int i = 0; i < CONNECT_RETRIES; i++) {

			try {
				socket.connect(sockAddr);
				break;
			} catch (ConnectException ce) {
				Thread.sleep(waitTime);
				waitTime *= 2;
			}
		}

		if (!socket.isConnected()) {
			throw new IOException("can't connect to SUMO server");
		}else{
			this.cp = new CommandProcessor(socket);
		}
		
	}

	/**
	 * Adds a custom option to the SUMO command line before executing it.
	 * 
	 * @param option
	 *            the option name, in long form (e.g. &quot;no-warnings&quot;
	 *            instead of &quot;W&quot;) and without initial dashes
	 * @param value
	 *            the option value, or <code>null</code> if the option has no
	 *            value
	 */
	public void addOption(String option, String value) {
		args.add("--" + option);
		if (value != null)
			args.add(value);
	}
	
	/**
	 * Runs a SUMO instance and tries to connect at it.
	 * 
	 * @throws IOException
	 *             if something wrong occurs while starting SUMO or connecting
	 *             at it.
	 */
	public void runServer() throws IOException {
		
		
		
		if(!this.remote){
		
		findAvailablePort();

		runSUMO();

		int waitTime = 500; // milliseconds
		try {
			for (int i = 0; i < CONNECT_RETRIES; i++) {
			

				socket = new Socket();
				socket.setTcpNoDelay(true);
				
				try {
					socket.connect(new InetSocketAddress("127.0.0.1", remotePort));
					break;
				} catch (ConnectException ce) {
					Thread.sleep(waitTime);
					waitTime *= 2;
				}
				
				
				
				
			}

			if (!socket.isConnected()) {
				throw new IOException("can't connect to SUMO server");
			}else{
				this.cp = new CommandProcessor(socket);
			}

		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		closeQuery = new CloseQuery(socket);
		
		}
		
	}

	private void runSUMO() throws IOException {
		
		args.add(0, sumoEXE);
		
		if(this.configFile != null){
		args.add("-c");
		args.add(configFile);
		}else if(this.net_file != null && this.route_file != null){
			
			args.add("--net-file");
			args.add(this.net_file);
			args.add("--route-files");
			args.add(this.route_file);
			
		}else{
			args.add("--net-file");
			args.add(this.net_file);
		}
		
		args.add("--remote-port");
		args.add(Integer.toString(remotePort));
		
		if (randomSeed != -1) {
			args.add("--seed");
			args.add(Integer.toString(randomSeed));
		}

		String[] argsArray = new String[args.size()];
		args.toArray(argsArray);
		
		
		sumoProcess = Runtime.getRuntime().exec(argsArray);

		

		StreamLogger errStreamLogger = new StreamLogger(sumoProcess.getErrorStream(), "SUMO-err:");
		StreamLogger outStreamLogger = new StreamLogger(sumoProcess.getInputStream(), "SUMO-out:");
		new Thread(errStreamLogger, "StreamLogger-SUMO-err").start();
		new Thread(outStreamLogger, "StreamLogger-SUMO-out").start();
	}

	private void findAvailablePort() throws IOException {
		ServerSocket testSock = new ServerSocket(0);
		remotePort = testSock.getLocalPort();
		testSock.close();
		testSock = null;
	}

	/**
	 * Closes the connection, quits the simulator, frees any stale
	 * resource and makes all {@link Vehicle} instances inactive.
	 * 
	 */

	public void close(){
		try {
			socket.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Closes the connection, eating the {@link InterruptedException} it may
	 * throw, hoping that Murphy's Law doesn't notice all this ugly thing.
	 */
	private void closeAndDontCareAboutInterruptedException() {
		close();
	}
	
	/**
	 * Returns <code>true</code> if the connection was closed by the user, or if
	 * an {@link IOException} was thrown after the connection was made.
	 * @see #close()
	 * @return boolean
	 */
	public boolean isClosed() {
		return socket == null || socket.isClosed();
	}

	
	public synchronized void do_job_set(SumoCommand cmd) throws Exception{
		
		if (isClosed())
			throw new IllegalStateException("connection is closed");
		
		try {this.cp.do_job_set(cmd);}
		catch (Exception e) {
			closeAndDontCareAboutInterruptedException();
			throw e;
		}
		
	}
	
	
	public synchronized Object do_job_get(SumoCommand cmd) throws Exception{
		
		Object output = null;
		if (isClosed())
			throw new IllegalStateException("connection is closed");
		
		try {
			output = this.cp.do_job_get(cmd);
		}
		catch (Exception e) {
			closeAndDontCareAboutInterruptedException();
			throw e;
		}
		
		return output;
	}
	
	
	public synchronized void do_timestep() throws Exception{
		
		if (isClosed())
			throw new IllegalStateException("connection is closed");
		
		try {this.cp.do_job_set(new SumoCommand(Constants.CMD_SIMSTEP2, 0));}
		catch (Exception e) {
			closeAndDontCareAboutInterruptedException();
			throw e;
		}
		
	}
	
}

